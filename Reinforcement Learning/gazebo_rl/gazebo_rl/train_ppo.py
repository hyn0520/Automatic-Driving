import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import torch
from stable_baselines3.common.callbacks import EvalCallback
from gazebo_rl.gazebo_env import GazeboCarEnv
import os
import optuna
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage, VecVideoRecorder
from stable_baselines3.common.callbacks import CheckpointCallback
import wandb
from wandb.integration.sb3 import WandbCallback
import gym


def evaluate_model(model, num_episodes=2):
    total_reward = 0.0
    for _ in range(num_episodes):
        obs = model.env.reset()
        episode_reward = 0.0
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = model.env.step(action)
            episode_reward += reward
        total_reward += episode_reward
        print("num_episodes:", num_episodes)
    return total_reward / num_episodes

# Optuna超参数优化
def optimize_hyperparams(trial):
    env = GazeboCarEnv(init_node=False)  # 为每个试验创建独立环境
    hyperparams = {
        "learning_rate": trial.suggest_loguniform("learning_rate", 1e-5, 1e-3),
        "batch_size": trial.suggest_categorical("batch_size", [32, 64, 128]),
        "n_steps": trial.suggest_categorical("n_steps", [1024, 2048]),  # 移除小值
        "gamma": trial.suggest_uniform("gamma", 0.9, 0.999),
        "ent_coef": trial.suggest_loguniform("ent_coef", 0.0001, 0.1),
        "device": "cuda" if torch.cuda.is_available() else "cpu"
    }
    
    model = PPO(
        policy="CnnPolicy",
        env=env,
        verbose=1,
        **{k: v for k, v in hyperparams.items() if k != 'device'}
    )
    best_metric = -float("inf")
    total_steps = 30000
    eval_interval = 10000
    
    for step in range(0, total_steps, eval_interval):
        model.learn(total_timesteps=eval_interval, progress_bar=True, log_interval=20)
        current_metric = evaluate_model(model)
        print("model is evaluated")
        # 报告并检查早停
        trial.report(current_metric, step)
        if trial.should_prune():
            env.close()
            print(f"Trial {trial.number} pruned at step {step}")
            raise optuna.TrialPruned()
        
        # 保存当前最佳模型
        if current_metric > best_metric:
            best_metric = current_metric
            print("best model is saved")
            model.save(f"/home/yinan/simulation_ws/src/gazebo_rl/model/trial_{trial.number}_best.zip")
    env.close()
    return best_metric
    # model.learn(total_timesteps=15000, progress_bar=True, log_interval=20)
    # print("----------Start Evaluating--------------")
    # mean_reward = evaluate_model(model)

    # print("----------Finished Evaluating----------")
    # trial_model_path = f"/home/yinan/simulation_ws/src/gazebo_rl/model/trial_{trial.number}.zip"
    # model.save(trial_model_path)
    # env.close()  # 重要！关闭环境避免资源泄漏
    # return mean_reward
def make_env():
    env = GazeboCarEnv(init_node=False)
    env = Monitor(env)  # record stats such as returns
    return env
def main():
    # 创建必要目录
    rclpy.init()
    try:
        os.makedirs("/home/yinan/simulation_ws/src/gazebo_rl/logs", exist_ok=True)
        os.makedirs("/home/yinan/simulation_ws/src/gazebo_rl/model", exist_ok=True)
        model_path = "/home/yinan/simulation_ws/src/gazebo_rl/model/trial_0_best.zip"
        # # 1. 超参数优化
        #study = optuna.create_study(direction="maximize")
        #study.optimize(optimize_hyperparams, n_trials=4)  # 示例用3次试验，实际建议50+
        best_trial_model_path = f"/home/yinan/simulation_ws/src/gazebo_rl/model/best_model_300_15.02_morning.zip"
        run = wandb.init(
        project="gazebo_rl",
        name="PPO-Gazebo-Demo",           # 每次启动一个新run的名称
        config={                          # 这里可放置一些超参数
            "policy_type": "CnnPolicy",
            "total_timesteps": 100000,
            "env_name": "GazeboCarEnv"
        },
        sync_tensorboard=True,  # 自动同步SB3的tensorboard日志到WandB
        monitor_gym=True,       # 自动记录gym环境数据（若监控兼容）
        save_code=True          # 保存当前脚本代码到WandB
        )
        # # 2. 使用最佳参数训练最终模型
        env = GazeboCarEnv(init_node=False)
        check_env(env, warn=True)
        # eval_env = GazeboCarEnv(init_node=False)
        raw_eval_env = GazeboCarEnv(init_node=False)

        # 第一步：dummyVecEnv包装
        eval_env = DummyVecEnv([lambda: raw_eval_env])

        # 第二步：VecTransposeImage包装
        eval_env = VecTransposeImage(eval_env)
        model = PPO.load(
        best_trial_model_path,
        env=env,
        device="cuda" if torch.cuda.is_available() else "cpu",
        tensorboard_log=f"runs",
        n_steps = 4096,#2048
        batch_size = 512#128
        # 覆盖超参数确保一致性
        # **study.best_params
        )
        eval_callback = EvalCallback(
            eval_env,
            best_model_save_path="/home/yinan/simulation_ws/src/gazebo_rl/model",
            log_path="./logs",  
            eval_freq=20000,     
            deterministic=True,
            render=False,
            n_eval_episodes=2   
        )
        # checkpoint_callback = CheckpointCallback(
        #         save_freq=10000,  # 每20,000步保存一次检查点
        #         save_path="/home/yinan/simulation_ws/src/gazebo_rl/checkpoints",  # 检查点保存路径
        #         name_prefix="ppo_gazebo_car"  # 检查点文件名前缀
        #     )
        Wandb_callback=WandbCallback(
            verbose=1,                     
            log="all",  # 也可设置 "gradients" 等
        )
        # 训练模型
        print("start learning")
        model.learn(
            total_timesteps=run.config["total_timesteps"],  
            callback= [eval_callback, Wandb_callback],
            progress_bar=True,
            log_interval=50,
            
        )

        # 保存最终模型
        model.save("/home/yinan/simulation_ws/src/gazebo_rl/model/model_final13.02")
        env.close()
        
     
        
    finally:
        # 程序结束时关闭ROS 2
        rclpy.shutdown()

if __name__ == "__main__":
    main()





# def main():
#     # 1. 创建必要目录
#     os.makedirs("./logs", exist_ok=True)
#     os.makedirs("/home/yinan/simulation_ws/src/gazebo_rl/model", exist_ok=True)
#     if not rclpy.ok():
#         rclpy.init()
#     # 2. 创建并检查环境
#     # env = GazeboCarEnv()
#     # env = Monitor(env, filename=None)
#     # check_env(env, warn=True)
#     # 训练环境
#     train_env = GazeboCarEnv(init_node=False)
#     train_env = Monitor(train_env, filename=None)

#     # 评估环境（独立环境，包一层Monitor）
#     eval_env = GazeboCarEnv(init_node=False)
#     eval_env = Monitor(eval_env, filename=None)


#     # 3. 初始化 PPO 模型（可以根据需要修改默认超参数）
#     model = PPO(
#         policy="CnnPolicy",
#         env=train_env,
#         verbose=1,
#         learning_rate=3e-4,            # 默认学习率
#         batch_size=64,                # 默认batch大小
#         n_steps=2048,                 # 每次更新的 rollout 步数
#         gamma=0.99,                   # 折扣因子
#         ent_coef=0.01,                # 熵系数
#         device="cuda" if torch.cuda.is_available() else "cpu"
#     )


#     # 4. 配置评估回调
#     eval_callback = EvalCallback(
#         eval_env,
#         best_model_save_path="/home/yinan/simulation_ws/src/gazebo_rl/model",
#         log_path="./logs",      # 存放评估日志
#         eval_freq=1000,         # 每2000步评估一次
#         deterministic=True,
#         render=False,
#         n_eval_episodes=5       # 每次评估跑5个episode
#     )

#     # 5. 训练模型 100 万步
#     model.learn(
#         total_timesteps=500_000,
#         progress_bar=True,
#         callback=eval_callback,     # 必须添加此行
#     )

#     # 6. 保存最终模型并关闭环境
#     model.save("ppo_gazebo_car_final")
#     train_env.close()
#     eval_env.close()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
# train_ppo.py
# import gymnasium as gym
# from stable_baselines3 import PPO
# from stable_baselines3.common.env_util import make_vec_env
# from stable_baselines3.common.vec_env import DummyVecEnv
# from stable_baselines3.common.callbacks import CheckpointCallback
# from stable_baselines3.common.monitor import Monitor
# import os
# from gazebo_env import GazeboCarEnv  # 导入你的环境类

# # 参数配置
# TOTAL_TIMESTEPS = 500000           # 总训练步数
# CHECKPOINT_FREQ = 10000             # 模型保存频率
# LOG_DIR = "./logs/"                 # 日志目录
# CHECKPOINT_DIR = "./checkpoints/"   # 检查点目录
# SAVE_PATH = "./ppo_gazebo_car"      # 最终模型保存路径

# def main():
#     # 创建日志目录
#     os.makedirs(LOG_DIR, exist_ok=True)
#     os.makedirs(CHECKPOINT_DIR, exist_ok=True)

#     # 创建环境（单环境）
#     env = GazeboCarEnv(init_node=True)
#     env = Monitor(env, LOG_DIR)     # 包装Monitor记录训练数据
    
#     # 初始化PPO模型
#     model = PPO(
#         policy="CnnPolicy",          # 使用CNN处理图像输入
#         env=env,
#         learning_rate=3e-4,         # 学习率
#         n_steps=2048,               # 每次更新使用的步数
#         batch_size=64,              # 批量大小
#         n_epochs=10,                # 每次更新的epoch数
#         gamma=0.99,                 # 折扣因子
#         gae_lambda=0.95,            # GAE参数
#         clip_range=0.2,             # 剪切范围
#         clip_range_vf=None,         # 值函数剪切范围
#         ent_coef=0.0,               # 熵系数
#         vf_coef=0.5,                # 值函数系数
#         max_grad_norm=0.5,          # 梯度裁剪
#         verbose=1,                  # 输出训练信息
#         tensorboard_log=LOG_DIR,    # TensorBoard日志目录
#         device="cuda"               # 使用GPU加速
#     )

#     # 创建检查点回调
#     checkpoint_callback = CheckpointCallback(
#         save_freq=CHECKPOINT_FREQ,
#         save_path=CHECKPOINT_DIR,
#         name_prefix="ppo_model"
#     )

#     try:
#         # 开始训练
#         model.learn(
#             total_timesteps=TOTAL_TIMESTEPS,
#             callback=checkpoint_callback,
#             tb_log_name="ppo_run",
#             reset_num_timesteps=True,
#             progress_bar=True
#         )
#     except KeyboardInterrupt:
#         print("Training interrupted by user.")

#     # 保存最终模型
#     model.save(SAVE_PATH)
#     print(f"Model saved to {SAVE_PATH}")

#     # 关闭环境
#     env.close()

# if __name__ == "__main__":
#     main()