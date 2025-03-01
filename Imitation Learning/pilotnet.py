import os
import csv
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
import random
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import random_split

def process_img(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    block_size = 61
    const_subtrahend = -70
    small_blurred = cv2.resize(blurred, None, fx=0.25, fy=0.25)
    small_binary = cv2.adaptiveThreshold(
        small_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
        block_size, const_subtrahend
    )
    binary = cv2.resize(small_binary, (blurred.shape[1], blurred.shape[0]))
    homography_data = [
        [-3.038122819655312, -8.672877526884868, 2377.9484334015165], 
        [0.29213993514510084, -16.02172757573472, 2881.3325514309886], 
        [0.0004933373163602723, -0.02681487260493437, 1.0]
    ]
    homography = np.array(homography_data, dtype=np.float64)
    binary_eagle = cv2.warpPerspective(binary, homography, (640, 480))
    resized = cv2.resize(binary_eagle, (224, 224))
    normalized = resized / 255.0
    final_image = np.expand_dims(normalized, axis=0).astype(np.float32)
    return final_image

class BehaviorCloneDataset(Dataset):
    def __init__(self, csv_file, img_dir, transform=None):
        """
        Args:
            csv_file (str): CSV 文件路径 (如 'dataset/driving_log.csv')
            img_dir  (str): 图片文件夹路径 (如 'dataset/IMG')
            transform: torchvision.transforms，用于后续图像预处理 (裁剪、缩放、增强)
        """
        self.img_dir = img_dir
        self.transform = transform
        self.samples = []
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)  # 假设表头是 "center,steering"
            for row in reader:
                img_path = row['image_path'].strip()  # e.g. 'IMG/center_00001.jpg'
                angular_velocity = float(row['angular_velocity'].strip())
                #linear_velocity = float(row['linear_velocity'].strip())
                # 归一化处理 (示例)
                angular_velocity = angular_velocity / 300.0  # => [-1, +1]  (若原本是 -300~+300)
                #linear_velocity = linear_velocity / 800.0  # => [0, 1]   (若原本是 0~800)

                # 全路径:
                full_path = os.path.join(os.path.dirname(csv_file), img_path)
                self.samples.append((full_path, angular_velocity))


    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index, device = 'cpu'):
        img_path, angular_velocity = self.samples[index]
        # 读取图像 (BGR)
        bgr_img = cv2.imread(img_path)
        
        if bgr_img is None:
            raise ValueError(f"Image not found: {img_path}")
        processed_img = process_img(bgr_img)
        img_2d = processed_img[0, :, :,]
        resized_img = cv2.resize(img_2d,(240,120))
        resized_img_final = np.expand_dims(resized_img, axis=0)
        img_tensor = torch.from_numpy(resized_img_final).float()
        

        return img_tensor, torch.tensor([angular_velocity], dtype=torch.float32)



# transform_pipeline = T.Compose([
#     T.Resize((120,240)),
#     T.ToTensor(),
#     T.Normalize(mean=[0.5,0.5,0.5], 
#                 std=[0.5,0.5,0.5])
# ])

class PilotNet(nn.Module):
    def __init__(self):
        super(PilotNet, self).__init__()
        # 增加初始通道数，添加BatchNorm
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=5, stride=2),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),

            nn.Conv2d(32, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),

            nn.Conv2d(48, 64, kernel_size=5, stride=2),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),

            nn.Conv2d(64, 96, kernel_size=3, stride=2),
            nn.BatchNorm2d(96),
            nn.ReLU(inplace=True),

            nn.Conv2d(96, 128, kernel_size=3, stride=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )

        self.avgpool = nn.AdaptiveAvgPool2d((4, 4))  # 自适应池化到固定大小

        self.regressor = nn.Sequential(
            nn.Linear(128 * 4 * 4, 256),
            nn.ReLU(inplace=True),
            #nn.Dropout(0.1),

            nn.Linear(256, 64),
            nn.ReLU(inplace=True),
            #nn.Dropout(0.1),

            nn.Linear(64, 1)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.regressor(x)
        return x

def train(model, train_loader, val_loader, num_epochs=10, lr=1e-4, device='gpu'):
    model = model.to(device)
    criterion = nn.SmoothL1Loss()
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=0.01)
    # 学习率调度器: 当验证损失在3个epoch内没有改善时,降低学习率
    scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer,
        max_lr=lr,
        epochs=num_epochs,
        steps_per_epoch=len(train_loader),
        div_factor=25,  # 初始lr = max_lr/25
        final_div_factor=1000,  # 最终lr = 初始lr/1000
    )

    best_val_loss = float('inf')
    best_model_weights = None
    no_improve_epochs = 0

    for epoch in range(num_epochs):
        # 训练阶段
        model.train()
        running_loss = 0.0
        for i, (images, angles) in enumerate(train_loader):
            images = images.to(device)
            angles = angles.to(device)

            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, angles)
            loss.backward()

            optimizer.step()
            scheduler.step()

            running_loss += loss.item()

            if (i + 1) % 100 == 0:
                print(f"Epoch [{epoch + 1}/{num_epochs}], "
                      f"Step [{i + 1}/{len(train_loader)}], "
                      f"Loss: {loss.item():.4f}, "
                      f"LR: {optimizer.param_groups[0]['lr']:.6f}")
        epoch_loss = running_loss / len(train_loader)
        print(f"Epoch [{epoch+1}/{num_epochs}] Training Loss: {epoch_loss:.5f}")

        # 验证阶段
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for images, angles in val_loader:
                images = images.to(device)
                angles = angles.to(device)
                outputs = model(images)
                loss = criterion(outputs, angles)
                val_loss += loss.item()
        val_loss /= len(val_loader)
        print(f"Epoch [{epoch + 1}/{num_epochs}] "
              f"Training Loss: {epoch_loss:.5f}, "
              f"Validation Loss: {val_loss:.5f}, "
              f"LR: {optimizer.param_groups[0]['lr']:.6f}")




        # 保存最佳模型
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_model_weights = model.state_dict().copy()
            no_improve_epochs = 0
        else:
            no_improve_epochs += 1

        # 早停: 如果连续10个epoch验证损失没有改善,则停止训练
        if no_improve_epochs >= 5:
            print(f"Early stopping triggered after epoch {epoch + 1}")
            break

    # 恢复最佳模型权重
    if best_model_weights is not None:
        model.load_state_dict(best_model_weights)
        print(f"Best validation loss: {best_val_loss:.5f}")

    return model

def evaluate_model(model, dataset, device='cpu'):
    """
    对模型进行推断，只对角速度(第一个输出)做对比。
    - 将预测结果(归一化到 [-1,1])反归一化到 [-300,300]
    - 将 dataset 中的真实标签也反归一化到 [-300,300]
    - 计算最大偏差、平均偏差等
    """
    model.eval()
    
    # 存放真实值和预测值
    all_labels = []
    all_preds = []

    with torch.no_grad():
        for i in range(len(dataset)):
            # 从 dataset 获取单个样本
            img, label = dataset[i]  # label = [angular_velocity_norm, linear_velocity_norm]
            
            # label 是归一化后的角速度 => label[0] 即角速度归一化
            # 要得到原始角速度值，需要反归一化 => 角速度原始范围 [-300, 300]
            ang_label_norm = label[0].item()
            ang_label = ang_label_norm * 300.0  # 反归一化

            # 将图像扩展 batch 维度并放到 device
            img = img.unsqueeze(0).to(device)

            # 前向传播得到预测值 (batch_size=1, 输出size=2)
            output = model(img)
            ang_pred_norm = output[0].item()  # 第一个输出=角速度(归一化)
            ang_pred = ang_pred_norm * 300.0      # 反归一化

            all_labels.append(ang_label)
            all_preds.append(ang_pred)
    
    # 计算偏差
    all_labels = np.array(all_labels)
    all_preds = np.array(all_preds)
    diffs = all_preds - all_labels

    max_diff = np.max(np.abs(diffs))  # 最大绝对偏差
    mean_diff = np.mean(np.abs(diffs))  # 平均绝对偏差

    print(f"共预测 {len(diffs)} 条样本")
    print(f"角速度 - 最大偏差: {max_diff:.2f}")
    print(f"角速度 - 平均偏差: {mean_diff:.2f}")

    # 如果需要返回便于后续可视化/分析
    return all_preds, all_labels, diffs

if __name__ == "__main__":
    #构造网络
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print("Using device:", device)

   
    net = PilotNet().to(device)

    checkpoint_path = "/home/yinan/PSAF/Behaviour Clone/pilotnetonly800_850.pth"
    net.load_state_dict(torch.load(checkpoint_path, map_location=device), strict=False)
    
    dataset = BehaviorCloneDataset(
        csv_file='/home/yinan/PSAF/Behaviour Clone/output_1000/synchronized_data.csv',
        img_dir = '/home/yinan/PSAF/Behaviour Clone/output_1000',
        transform=None
    )
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=64, shuffle=False)
    # 训练
    net = train(
        net,
        train_loader,
        val_loader,
        num_epochs=30,  # 增加训练轮数
        lr=1e-3,  # 使用更大的初始学习率
        device=device
    )

    # 保存最终模型
    torch.save(net.state_dict(), "/home/yinan/PSAF/Behaviour Clone/pilotnetonly800_850_1000.pth")

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = PilotNet().to(device)
    
    # 2. 加载你之前训练好的权重
    model.load_state_dict(torch.load("/home/yinan/PSAF/Behaviour Clone/pilotnetonly800_850_1000.pth", map_location=device))
    
    # 3. 构建同样的 Dataset (或单独的测试集)
    dataset = BehaviorCloneDataset(
        csv_file='/home/yinan/PSAF/Behaviour Clone/output_1000/synchronized_data.csv',
        img_dir = '/home/yinan/PSAF/Behaviour Clone/output_1000',
        transform=None
    )

    # 4. 做推断并计算偏差
    preds, labels, diffs = evaluate_model(model, dataset, device=device)

    # 你也可以在这里进一步分析，如打印前10条的预测与真实值：
    print("\n前10条预测 vs 真实：")
    for i in range(10):
        print(f"预测 = {preds[i]:.2f}, 真实 = {labels[i]:.2f}, 差值 = {diffs[i]:.2f}")
