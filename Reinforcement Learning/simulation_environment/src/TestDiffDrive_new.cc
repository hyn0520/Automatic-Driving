// TestDiffDrive.cc (Gazebo Garden 版示例)

#include "TestDiffDrive.hh"

// --------------------------
// 1) 新的 Gazebo Garden / gz-sim7 头文件
// --------------------------
#include <gz/plugin/Register.hh>
#include <gz/common5/Profiler.hh>        // 原先 "ignition/common/Profiler.hh" -> "gz/common5/Profiler.hh"
#include <gz/math7/PID.hh>              // 原先 "ignition/math/PID.hh" -> "gz/math7/PID.hh"
#include <gz/msgs9/double.pb.h>         // 原先 "ignition/msgs/double.pb.h" -> "gz/msgs9/double.pb.h"
#include <gz/sim7/Model.hh>             // 原先 "ignition/gazebo/Model.hh" 
#include <gz/sim7/Util.hh>              // 原先 "ignition/gazebo/Util.hh" (如需要)
#include <gz/sim7/components/JointForceCmd.hh>
#include <gz/sim7/components/JointPosition.hh>
#include <gz/sim7/components/JointVelocity.hh>
#include <gz/sim7/components/JointVelocityCmd.hh>
#include <gz/sim7/EntityComponentManager.hh>
#include <gz/sim7/Events.hh>
#include <gz/sim7/Types.hh>             // kNullEntity, etc.
#include <gz/sim7/UpdateInfo.hh>        
#include <gz/transport12/Node.hh>       // 原先 "ignition/transport/Node.hh" -> "gz/transport12/Node.hh"
#include <gz/transport12/TopicUtils.hh> // 原先 "ignition/transport/TopicUtils.hh"

// --------------------------
// 2) 额外的第三方 / ROS 头文件 (如果仍需要)
// --------------------------
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <iostream>

// 将常用命名空间引入
using namespace gz;
using namespace sim;
using namespace gz::sim::v7;    // Garden 对应 v7 inline namespace
using namespace std::chrono_literals;

// --------------------------
// 3) SampleSystemPrivate 类
// --------------------------
class SampleSystemPrivate
{
  public: SampleSystemPrivate() = default;

  // 速度控制回调
  public: void OnCmdVel(const gz::msgs::Double &_msg)
  {
    std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
    // 这里的 wheelDiameter 在 private 中定义
    this->jointVelCmd = ((float)_msg.data() * (4 / 1500.0) / this->wheelDiameter) * 0.55;
  }

  // 转向控制回调
  public: void OnCmdVel2(const gz::msgs::Double &_msg)
  {
    std::lock_guard<std::mutex> lock(this->jointVelCmdMutex2);
    if (std::abs(_msg.data()) > 0 )
      this->jointVelCmd2 = 0.78535f * (float)_msg.data() / 800.f * 0.45f;
    else
      this->jointVelCmd2 = 0.0;
  }

  // --------------------------
  // 以下成员为你的老代码中出现的各种变量
  // --------------------------
  public: gz::transport::Node node;           // 原 ignition::transport::Node

  // Joint entities
  public: Entity jointEntity     {kNullEntity};
  public: Entity jointEntity2    {kNullEntity};
  public: Entity jointEntity3    {kNullEntity};
  public: Entity jointEntity4    {kNullEntity};
  public: Entity frontWheelLeftJointEntity  {kNullEntity};
  public: Entity frontWheelRightJointEntity {kNullEntity};
  public: Entity wheelEntity {kNullEntity};

  // Joint names
  public: std::string jointName;
  public: std::string jointName2;
  public: std::string jointName3;
  public: std::string jointName4;
  public: std::string frontWheelLeftJointName;
  public: std::string frontWheelRightJointName;

  // 速度命令
  public: double jointVelCmd       {0.0};
  public: double jointVelCmd2      {0.0};
  public: double jointVelCmd3      {0.0};
  public: double jointVelCmd4      {0.0};
  public: double jointVelCmdFrontLeft  {0.0};
  public: double jointVelCmdFrontRight {0.0};

  public: std::mutex jointVelCmdMutex;
  public: std::mutex jointVelCmdMutex2;
  public: std::mutex jointVelCmdMutex3;
  public: std::mutex jointVelCmdMutex4;
  public: std::mutex jointVelCmdMutexFrontLeft;
  public: std::mutex jointVelCmdMutexFrontRight;

  // Model
  public: Model model{kNullEntity};

  // 轮子 / 车辆参数
  public: double wheelDistance {0.2};
  public: double axelDistance  {0.25};
  public: double wheelDiameter {0.03};
  public: std::string carType {"old"};

  // 是否使用力模式
  public: bool useForceCommands {false};

  // PID
  public: gz::math::PID velPid;
  public: gz::math::PID posPid;
  public: gz::math::PID posPid2;
  public: gz::math::PID posPidInner;
  public: gz::math::PID posPid2Inner;

  // Joint index
  public: unsigned int jointIndex = 0u;

  // 计数器 / 速度值
  public: double counter     {0.0};
  public: double speedValue {0.0};

  // 话题
  public: std::string topic_pose{"/dt_data"};
  public: gz::transport::Node::Publisher pub_pose =
            node.Advertise<gz::msgs::Double>(topic_pose);

  public: std::string topic_hall_8{"/dt8_data"};
  public: gz::transport::Node::Publisher pub_hall_8 =
            node.Advertise<gz::msgs::Double>(topic_hall_8);

  public: std::string topic_hall_counter{"/hall_cnt_data"};
  public: gz::transport::Node::Publisher pub_hall_cnt =
            node.Advertise<gz::msgs::Double>(topic_hall_counter);

  public: std::string topic_increment{"/enc_cm_data"};
  public: gz::transport::Node::Publisher pub_enc_data =
            node.Advertise<gz::msgs::Double>(topic_increment);

  public: std::string topic_step{"/enc_step_data"};
  public: gz::transport::Node::Publisher pub_step_data =
            node.Advertise<gz::msgs::Double>(topic_step);
};

// --------------------------
// 4) SampleSystem 本体
// --------------------------
SampleSystem::SampleSystem()
  : dataPtr(std::make_unique<SampleSystemPrivate>())
{
  // 可以在此做一些初始化
}

// Configure 回调
void SampleSystem::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager & /*_eventMgr*/)
{
  // 加入 Profiling
  GZ_PROFILE("SampleSystem::Configure");

  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "[SampleSystem] Plugin should be attached to a valid model. "
          << "Failed to initialize.\n";
    return;
  }

  // 从 <sdf> 中读取若干参数
  if (_sdf->HasElement("joint_name"))
    this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");
  if (_sdf->HasElement("joint_name_2"))
    this->dataPtr->jointName2 = _sdf->Get<std::string>("joint_name_2");
  if (_sdf->HasElement("joint_name_3"))
    this->dataPtr->jointName3 = _sdf->Get<std::string>("joint_name_3");
  if (_sdf->HasElement("joint_name_4"))
    this->dataPtr->jointName4 = _sdf->Get<std::string>("joint_name_4");
  if (_sdf->HasElement("front_wheel_left"))
    this->dataPtr->frontWheelLeftJointName = _sdf->Get<std::string>("front_wheel_left");
  if (_sdf->HasElement("front_wheel_right"))
    this->dataPtr->frontWheelRightJointName = _sdf->Get<std::string>("front_wheel_right");

  // 其它参数
  this->dataPtr->wheelDistance = _sdf->Get<double>("wheel_distance", 0.2).first;
  this->dataPtr->axelDistance  = _sdf->Get<double>("axel_distance", 0.25).first;
  this->dataPtr->wheelDiameter = _sdf->Get<double>("wheel_radius", 0.03).first;
  this->dataPtr->carType       = _sdf->Get<std::string>("car_type", "old").first;

  // 初始化计数器
  this->dataPtr->counter = 0.0;

  // PID 初始化示例
  double p_inner  = 1000.0;
  double i_inner  = 2.0;
  double d_inner  = this->dataPtr->wheelDistance;
  double i_max_in = 1.0;
  double i_min_in = 1.0;
  double cmd_max_in = 1000.0;
  double cmd_min_in = -1000.0;
  double cmd_offset_in = 0.0;

  this->dataPtr->posPidInner.Init(p_inner, i_inner, d_inner,
                                  i_max_in, i_min_in,
                                  cmd_max_in, cmd_min_in,
                                  cmd_offset_in);
  this->dataPtr->posPid2Inner.Init(p_inner, i_inner, d_inner,
                                   i_max_in, i_min_in,
                                   cmd_max_in, cmd_min_in,
                                   cmd_offset_in);

  // 从 SDF 获取其它 PID 参数
  double p         = _sdf->Get<double>("p_gain",     1.0).first;
  double i         = _sdf->Get<double>("i_gain",     0.0).first;
  double d         = _sdf->Get<double>("d_gain",     0.0).first;
  double iMax      = _sdf->Get<double>("i_max",      1.0).first;
  double iMin      = _sdf->Get<double>("i_min",     -1.0).first;
  double cmdMax    = _sdf->Get<double>("cmd_max",    1000.0).first;
  double cmdMin    = _sdf->Get<double>("cmd_min",   -1000.0).first;
  double cmdOffset = _sdf->Get<double>("cmd_offset", 0.0).first;

  // 是否 force 模式
  if (_sdf->HasElement("use_force_commands") &&
      _sdf->Get<bool>("use_force_commands"))
  {
    this->dataPtr->useForceCommands = true;
    this->dataPtr->velPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

    gzdbg << "[SampleSystem] Force mode with PID params:\n"
          << "p_gain: "     << p         << "\n"
          << "i_gain: "     << i         << "\n"
          << "d_gain: "     << d         << "\n"
          << "i_max: "      << iMax      << "\n"
          << "i_min: "      << iMin      << "\n"
          << "cmd_max: "    << cmdMax    << "\n"
          << "cmd_min: "    << cmdMin    << "\n"
          << "cmd_offset: " << cmdOffset << std::endl;
  }
  else
  {
    gzmsg << "[SampleSystem] Velocity mode.\n";
  }

  // 初始化位置 PID
  this->dataPtr->posPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->posPid2.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  // --------------------------
  // 订阅话题
  // --------------------------
  std::string topic = gz::transport::TopicUtils::AsValidTopic("/set_motor_level_msg");
  if (topic.empty())
  {
    gzerr << "Failed to create topic for joint [" << this->dataPtr->jointName
          << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic,
      &SampleSystemPrivate::OnCmdVel, this->dataPtr.get());
  gzmsg << "[SampleSystem] subscribing to Double msgs on [" << topic << "]\n";

  std::string topic2 = gz::transport::TopicUtils::AsValidTopic("/set_steering_level_msg");
  if (topic2.empty())
  {
    gzerr << "Failed to create topic for joint [" << this->dataPtr->jointName2
          << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic2,
      &SampleSystemPrivate::OnCmdVel2, this->dataPtr.get());
  gzmsg << "[SampleSystem] subscribing to Double msgs on [" << topic2 << "]\n";
}

// PreUpdate 回调
void SampleSystem::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  GZ_PROFILE("SampleSystem::PreUpdate");

  // 检测是否倒退
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly.\n";
  }

  // 如果还没拿到 Joint，先通过 JointByName 查找
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
       this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }
  if (this->dataPtr->jointEntity3 == kNullEntity)
  {
    this->dataPtr->jointEntity3 =
       this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName3);
  }
  if (this->dataPtr->jointEntity2 == kNullEntity)
  {
    this->dataPtr->jointEntity2 =
       this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName2);
  }
  if (this->dataPtr->jointEntity4 == kNullEntity)
  {
    this->dataPtr->jointEntity4 =
       this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName4);
  }
  if (this->dataPtr->frontWheelLeftJointEntity == kNullEntity)
  {
    this->dataPtr->frontWheelLeftJointEntity =
       this->dataPtr->model.JointByName(_ecm, this->dataPtr->frontWheelLeftJointName);
  }
  if (this->dataPtr->frontWheelRightJointEntity == kNullEntity)
  {
    this->dataPtr->frontWheelRightJointEntity =
       this->dataPtr->model.JointByName(_ecm, this->dataPtr->frontWheelRightJointName);
  }
  if (this->dataPtr->wheelEntity == kNullEntity)
  {
    this->dataPtr->wheelEntity =
       this->dataPtr->model.LinkByName(_ecm, "left_wheel_back");
  }

  // 如果还没找到有效关节，就退出
  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // 若暂停就不做
  if (_info.paused)
    return;

  // 获取/创建相关组件
  auto jointVelComp =
      _ecm.Component<components::JointVelocity>(this->dataPtr->jointEntity);
  auto jointVelComp3 =
      _ecm.Component<components::JointVelocity>(this->dataPtr->jointEntity3);
  auto jointVelCmdFrontLeft =
      _ecm.Component<components::JointVelocity>(this->dataPtr->frontWheelLeftJointEntity);
  auto jointVelCmdFrontRight =
      _ecm.Component<components::JointVelocity>(this->dataPtr->frontWheelRightJointEntity);
  auto jointPosComp =
      _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity2);
  auto jointPosComp2 =
      _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity4);

  if (!jointVelComp && !jointPosComp && !jointPosComp2 && !jointVelComp3 &&
      !jointVelCmdFrontLeft && !jointVelCmdFrontRight)
  {
    _ecm.CreateComponent(
       this->dataPtr->jointEntity, components::JointVelocity());
    _ecm.CreateComponent(
       this->dataPtr->jointEntity3, components::JointVelocity());
    _ecm.CreateComponent(
       this->dataPtr->frontWheelLeftJointEntity, components::JointVelocity());
    _ecm.CreateComponent(
       this->dataPtr->frontWheelRightJointEntity, components::JointVelocity());
    _ecm.CreateComponent(
       this->dataPtr->jointEntity2, components::JointPosition());
    _ecm.CreateComponent(
       this->dataPtr->jointEntity4, components::JointPosition());
  }

  if (!jointVelComp && !jointPosComp && !jointPosComp2 && !jointVelComp3 &&
      !jointVelCmdFrontLeft && !jointVelCmdFrontRight)
    return;

  // 若 jointIndex 越界，也直接返回
  if (jointPosComp && this->dataPtr->jointIndex >= jointPosComp->Data().size())
  {
    static bool invalidReported = false;
    if (!invalidReported)
    {
      gzerr << "[SampleSystem]: Invalid <joint_index> ["
            << this->dataPtr->jointIndex << "] bigger than ["
            << jointPosComp->Data().size() << "].\n";
      invalidReported = true;
    }
    return;
  }

  double targetVelLeft   = 0.0;
  double targetVelRight  = 0.0;
  double targetVelFrontLeft  = 0.0;
  double targetVelFrontRight = 0.0;
  double targetVel = 0.0;
  double position  = 0.0;

  // 计算各种转向 / 速度
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex);

    // 这里用 jointVelCmd2 来表示转向角？
    targetVelLeft  =
      this->dataPtr->jointVelCmd * (1 + (this->dataPtr->wheelDistance *
                           std::tan(this->dataPtr->jointVelCmd2)) / (2.0 * this->dataPtr->axelDistance));
    targetVelRight =
      this->dataPtr->jointVelCmd * (1 - (this->dataPtr->wheelDistance *
                           std::tan(this->dataPtr->jointVelCmd2)) / (2.0 * this->dataPtr->axelDistance));

    if (std::abs(this->dataPtr->jointVelCmd2) > 1e-6)
    {
      targetVelFrontLeft =
         targetVelLeft * (
           std::sqrt(std::pow(this->dataPtr->axelDistance /
                     std::tan(this->dataPtr->jointVelCmd2) - this->dataPtr->wheelDistance/2, 2)
                     + std::pow(this->dataPtr->axelDistance, 2))
           / std::sqrt(std::pow(this->dataPtr->axelDistance /
                     std::tan(this->dataPtr->jointVelCmd2) - this->dataPtr->wheelDistance/2, 2)) );

      targetVelFrontRight =
         targetVelRight * (
           std::sqrt(std::pow(this->dataPtr->axelDistance /
                     std::tan(this->dataPtr->jointVelCmd2) + this->dataPtr->wheelDistance/2, 2)
                     + std::pow(this->dataPtr->axelDistance, 2))
           / std::sqrt(std::pow(this->dataPtr->axelDistance /
                     std::tan(this->dataPtr->jointVelCmd2) + this->dataPtr->wheelDistance/2, 2)) );
    }
    else
    {
      targetVelFrontLeft  = this->dataPtr->jointVelCmd;
      targetVelFrontRight = this->dataPtr->jointVelCmd;
    }

    targetVel = this->dataPtr->jointVelCmd;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex2);
    position = this->dataPtr->jointVelCmd2;
  }

  // speedValue = 1 / jointVelComp->Data().at(0);
  if (jointVelComp && !jointVelComp->Data().empty())
  {
    this->dataPtr->speedValue = 1.0 / jointVelComp->Data().at(0);
  }

  // 如果是 Force 模式 ...
  if (this->dataPtr->useForceCommands)
  {
    // TODO: 你原本的力模式逻辑
  }
  else
  {
    // Velocity mode: 写 JointVelocityCmd
    auto vel =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->jointEntity);
    if (!vel)
    {
      _ecm.CreateComponent(
         this->dataPtr->jointEntity,
         components::JointVelocityCmd({targetVelLeft}));
    }
    else
    {
      vel->Data()[0] = targetVelLeft;
    }

    auto vel3 =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->jointEntity3);
    if (!vel3)
    {
      _ecm.CreateComponent(
         this->dataPtr->jointEntity3,
         components::JointVelocityCmd({targetVelRight}));
    }
    else
    {
      vel3->Data()[0] = targetVelRight;
    }

    auto velLeftFront =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->frontWheelLeftJointEntity);
    if (!velLeftFront)
    {
      _ecm.CreateComponent(
         this->dataPtr->frontWheelLeftJointEntity,
         components::JointVelocityCmd({targetVelFrontLeft}));
    }
    else
    {
      velLeftFront->Data()[0] = targetVelFrontLeft;
    }

    auto velRightFront =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->frontWheelRightJointEntity);
    if (!velRightFront)
    {
      _ecm.CreateComponent(
         this->dataPtr->frontWheelRightJointEntity,
         components::JointVelocityCmd({targetVelFrontRight}));
    }
    else
    {
      velRightFront->Data()[0] = targetVelFrontRight;
    }

    // 这里是你额外的 "force" 计算 / PIDs
    if (jointPosComp && jointPosComp2)
    {
      double error = 0.0;
      double error2 = 0.0;
      double error_inner = 0.0;
      double error2_inner = 0.0;

      {
        std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex2);
        if (!jointPosComp->Data().empty())
        {
          error = jointPosComp->Data().at(this->dataPtr->jointIndex) -
                  std::atan( (2.0 * this->dataPtr->axelDistance *
                              std::sin(this->dataPtr->jointVelCmd2))
                            /(2.0 * this->dataPtr->axelDistance *
                                std::cos(this->dataPtr->jointVelCmd2)
                              + this->dataPtr->wheelDistance *
                                std::sin(this->dataPtr->jointVelCmd2)) );
        }
      }
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex4);
        if (!jointPosComp2->Data().empty())
        {
          error2 = jointPosComp2->Data().at(this->dataPtr->jointIndex) -
                   std::atan( (2.0 * this->dataPtr->axelDistance *
                               std::sin(this->dataPtr->jointVelCmd2))
                             /(2.0 * this->dataPtr->axelDistance *
                                 std::cos(this->dataPtr->jointVelCmd2)
                               - this->dataPtr->wheelDistance *
                                 std::sin(this->dataPtr->jointVelCmd2)) );
        }
      }

      double force  = this->dataPtr->posPid.Update(error, _info.dt);
      double force2 = this->dataPtr->posPid2.Update(error2, _info.dt);

      error_inner  = jointPosComp->Data().at(this->dataPtr->jointIndex) - force;
      error2_inner = jointPosComp2->Data().at(this->dataPtr->jointIndex) - force2;

      double force_inner  = this->dataPtr->posPidInner.Update(error_inner, _info.dt);
      double force2_inner = this->dataPtr->posPid2Inner.Update(error2_inner, _info.dt);

      auto forceComp =
         _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity2);
      auto forceComp2 =
         _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity4);

      if (!forceComp)
      {
        _ecm.CreateComponent(this->dataPtr->jointEntity2,
                             components::JointForceCmd({force_inner}));
      }
      else
      {
        if (forceComp->Data().size() > this->dataPtr->jointIndex)
          forceComp->Data()[this->dataPtr->jointIndex] = force_inner;
      }

      if (!forceComp2)
      {
        _ecm.CreateComponent(this->dataPtr->jointEntity4,
                             components::JointForceCmd({force2_inner}));
      }
      else
      {
        if (forceComp2->Data().size() > this->dataPtr->jointIndex)
          forceComp2->Data()[this->dataPtr->jointIndex] = force2_inner;
      }
    }
  } // end else (velocity mode)
}

// PostUpdate 回调
void SampleSystem::PostUpdate(const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
{
  GZ_PROFILE("SampleSystem::PostUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly.\n";
  }

  // 这里是你原有的 "publish sensor" 逻辑
  if (this->dataPtr->carType == "old")
  {
    gz::msgs::Double hall;
    gz::msgs::Double hall8;
    gz::msgs::Double hallCnt;

    // 更新 counter
    this->dataPtr->counter += 1.0;
    // speedValue 就是 1 / jointVel(0)，上面 preupdate 里赋值

    hall.set_data(this->dataPtr->speedValue);
    hallCnt.set_data(this->dataPtr->counter);

    this->dataPtr->pub_pose.Publish(hall);
    this->dataPtr->pub_hall_cnt.Publish(hallCnt);

    if (fmod(this->dataPtr->counter, 8.0) == 0.0)
    {
      hall8.set_data(this->dataPtr->speedValue * 8.0);
      this->dataPtr->pub_hall_8.Publish(hall8);
    }
  }
  else
  {
    gz::msgs::Double encData;
    gz::msgs::Double encStep;

    this->dataPtr->counter += 1.0;
    encData.set_data(this->dataPtr->speedValue * this->dataPtr->wheelDiameter);
    encStep.set_data(this->dataPtr->counter);

    this->dataPtr->pub_enc_data.Publish(encData);
    this->dataPtr->pub_step_data.Publish(encStep);
  }
  (void)_ecm;  // 如果没用，可以忽略
}

// --------------------------
// 5) 插件注册宏
// --------------------------
// 老的 IGNITION_ADD_PLUGIN(...) => GZ_ADD_PLUGIN(...)
GZ_ADD_PLUGIN(
  SampleSystem,
  System,
  SampleSystem::ISystemConfigure,
  SampleSystem::ISystemPreUpdate,
  SampleSystem::ISystemPostUpdate)

// 同理老的 IGNITION_ADD_PLUGIN_ALIAS => GZ_ADD_PLUGIN_ALIAS
GZ_ADD_PLUGIN_ALIAS(SampleSystem,
                    "gz::sim::systems::SampleSystem")
