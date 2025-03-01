// comment
#include <ignition/plugin/Register.hh>
#include "TestDiffDrive.hh"

#include <ignition/msgs/double.pb.h>

#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::SampleSystemPrivate
{
/// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Double &_msg);

  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel2(const ignition::msgs::Double &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

   /// \brief Joint Entity
  public: Entity jointEntity2;

  /// \brief Joint Entity
  public: Entity jointEntity3;

  /// \brief Joint Entity
  public: Entity jointEntity4;

  /// \brief Joint Entity
  public: Entity frontWheelLeftJointEntity;

  /// \brief Joint Entity
  public: Entity frontWheelRightJointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Second joint name
  public: std::string jointName2;

  /// \brief Second joint name
  public: std::string jointName3;

  /// \brief Second joint name
  public: std::string jointName4;

  /// \brief Joint for front wheel left
  public: std::string frontWheelLeftJointName;

  /// \brief Joint for front wheel right
  public: std::string frontWheelRightJointName;

  /// \brief Joint for front wheel right 

  /// \brief Commanded joint velocity
  public: double jointVelCmd;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief Commanded joint velocity
  public: double jointVelCmd2;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex2;

    /// \brief Commanded joint velocity
  public: double jointVelCmd3;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex3;

      /// \brief Commanded joint velocity
  public: double jointVelCmd4;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex4;

  /// \brief Commanded joint velocity
  public: double jointVelCmdFrontLeft;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutexFrontLeft;

  /// \brief Commanded joint velocity
  public: double jointVelCmdFrontRight;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutexFrontRight;

  /// \brief Model interface
  public: Model model{kNullEntity};

  public: Entity wheelEntity;

  /// \brief True if force commands are internally used to keep the target
  /// velocity.
  public: bool useForceCommands{false};

  /// \brief Velocity PID controller.
  public: ignition::math::PID velPid;

  /// \brief Position PID controller.
  public: ignition::math::PID posPid;

  /// \brief Position PID controller.
  public: ignition::math::PID posPid2;

  /// \brief Position PID controller.
  public: ignition::math::PID posPidInner;

  /// \brief Position PID controller.
  public: ignition::math::PID posPid2Inner;

  /// \brief Joint index to be used.
  public: unsigned int jointIndex = 0u;

  /// \brief Wheel distance
  public: double wheelDistance;

  /// \brief wheel diameter
  public: double wheelDiameter;

  /// \brief axel distance
  public: double axelDistance;

  /// \brief car type
  public: std::string carType;

  std::string topic_pose = "/dt_data";
  public: transport::Node::Publisher pub_pose = node.Advertise<ignition::msgs::Double>(topic_pose);

  std::string topic_hall_8 ="/dt8_data";
  public: transport::Node::Publisher pub_hall_8 = node.Advertise<ignition::msgs::Double>(topic_hall_8);

  std::string topic_hall_counter = "/hall_cnt_data";
  public: transport::Node::Publisher pub_hall_cnt = node.Advertise<ignition::msgs::Double>(topic_hall_counter);

  std::string topic_increment = "/enc_cm_data";
  public: transport::Node::Publisher pub_enc_data = node.Advertise<ignition::msgs::Double>(topic_increment);

  std::string topic_step = "/enc_step_data";
  public: transport::Node::Publisher pub_step_data = node.Advertise<ignition::msgs::Double>(topic_step);
 };

SampleSystem::SampleSystem()
  : dataPtr(std::make_unique<SampleSystemPrivate>())
{
}

//////////////////////////////////////////////////
void SampleSystem::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "JointController plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");
  this->dataPtr->jointName2 = _sdf->Get<std::string>("joint_name_2");
  this->dataPtr->jointName3 = _sdf->Get<std::string>("joint_name_3");
  this->dataPtr->jointName4 = _sdf->Get<std::string>("joint_name_4");
  this->dataPtr->frontWheelLeftJointName = _sdf->Get<std::string>("front_wheel_left");
  this->dataPtr->frontWheelRightJointName = _sdf->Get<std::string>("front_wheel_right");
  this->dataPtr->wheelDistance = _sdf->Get<double>("wheel_distance", 0.2).first;
  this->dataPtr->axelDistance = _sdf->Get<double>("axel_distance", 0.25).first;
  this->dataPtr->wheelDiameter = _sdf->Get<double>("wheel_radius", 0.03).first;
  this->dataPtr->carType = _sdf->Get<std::string>("car_type", "old").first; // enum ["old", "new"]

  if (this->dataPtr->jointName == "" || this->dataPtr->jointName2 == "")
  {
    ignerr << "JointController found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

    // Reset counter for hall sensor
    counter = 0;

    // PID parameters for position holfing
    double p_inner        = 1000.0;
    double i_inner        = 2.0;
    double d_inner        = this->dataPtr->wheelDistance;
    double i_max_inner    = 1.0;
    double i_min_inner    = 1.0;
    double cmd_max_inner  = 1000.0;
    double cmd_min_inner  = -1000.0;
    double cmd_offset_in  = 0.0;

    // PID parameters
    double p         = _sdf->Get<double>("p_gain",     1.0).first;
    double i         = _sdf->Get<double>("i_gain",     0.0).first;
    double d         = _sdf->Get<double>("d_gain",     0.0).first;
    double iMax      = _sdf->Get<double>("i_max",      1.0).first;
    double iMin      = _sdf->Get<double>("i_min",     -1.0).first;
    double cmdMax    = _sdf->Get<double>("cmd_max",    1000.0).first;
    double cmdMin    = _sdf->Get<double>("cmd_min",   -1000.0).first;
    double cmdOffset = _sdf->Get<double>("cmd_offset", 0.0).first;

    this->dataPtr->posPidInner.Init(p_inner, i_inner, d_inner, i_max_inner, i_min_inner, cmd_max_inner, cmd_min_inner, cmd_offset_in);
    this->dataPtr->posPid2Inner.Init(p_inner, i_inner, d_inner, i_max_inner, i_min_inner, cmd_max_inner, cmd_min_inner, cmd_offset_in);

  if (_sdf->HasElement("use_force_commands") &&
      _sdf->Get<bool>("use_force_commands"))
  {
    this->dataPtr->useForceCommands = true;

    this->dataPtr->velPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

    igndbg << "[JointController] Force mode with parameters:" << std::endl;
    igndbg << "p_gain: ["     << p         << "]"             << std::endl;
    igndbg << "i_gain: ["     << i         << "]"             << std::endl;
    igndbg << "d_gain: ["     << d         << "]"             << std::endl;
    igndbg << "i_max: ["      << iMax      << "]"             << std::endl;
    igndbg << "i_min: ["      << iMin      << "]"             << std::endl;
    igndbg << "cmd_max: ["    << cmdMax    << "]"             << std::endl;
    igndbg << "cmd_min: ["    << cmdMin    << "]"             << std::endl;
    igndbg << "cmd_offset: [" << cmdOffset << "]"             << std::endl;
  }
  else
  {
    ignmsg << "[JointController] Velocity mode" << std::endl;
  }

  this->dataPtr->posPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->posPid2.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  // Subscribe to commands
  std::string topic = transport::TopicUtils::AsValidTopic("/set_motor_level_msg");
  if (topic.empty())
  {
    ignerr << "Failed to create topic for joint [" << this->dataPtr->jointName
           << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic, &SampleSystemPrivate::OnCmdVel,
                                this->dataPtr.get());

  ignmsg << "JointController subscribing to Double messages on [" << topic
         << "]" << std::endl;

  // Subscribe to commands
  std::string topic2 = transport::TopicUtils::AsValidTopic("/set_steering_level_msg");
  if (topic2.empty())
  {
    ignerr << "Failed to create topic for joint [" << this->dataPtr->jointName2
           << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic2, &SampleSystemPrivate::OnCmdVel2,
                                this->dataPtr.get());

  ignmsg << "JointController subscribing to Double messages on [" << topic2
         << "]" << std::endl;
}

void SampleSystem::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joint hasn't been identified yet, look for it
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

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Create joint velocity component if one doesn't exist
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
  if (jointVelComp == nullptr && jointPosComp == nullptr && jointPosComp == nullptr && jointVelComp3 == nullptr 
      && jointVelCmdFrontLeft == nullptr && jointVelCmdFrontRight == nullptr)
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
  if (jointVelComp == nullptr && jointPosComp == nullptr && jointPosComp == nullptr && jointVelComp3 == nullptr 
      && jointVelCmdFrontLeft == nullptr && jointVelCmdFrontRight == nullptr)
    return;

  // Sanity check: Make sure that the joint index is valid.
  if (this->dataPtr->jointIndex >= jointPosComp->Data().size())
  {
    static bool invalidJointReported = false;
    if (!invalidJointReported)
    {
      ignerr << "[JointPositionController]: Detected an invalid <joint_index> "
             << "parameter. The index specified is ["
             << this->dataPtr->jointIndex << "] but the joint only has ["
             << jointPosComp->Data().size() << "] index[es]. "
             << "This controller will be ignored" << std::endl;
      invalidJointReported = true;
    }
    return;
  }

  double targetVelLeft;
  double targetVelRight;
  double targetVelFrontLeft;
  double targetVelFrontRight;
  double targetVel;
  double position;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex);
    targetVelLeft = this->dataPtr->jointVelCmd * (1 + (this->dataPtr->wheelDistance* tan(this->dataPtr->jointVelCmd2))/(2.0 * this->dataPtr->axelDistance)) ;
    targetVelRight = this->dataPtr->jointVelCmd * (1 - (this->dataPtr->wheelDistance* tan(this->dataPtr->jointVelCmd2))/(2.0 * this->dataPtr->axelDistance)) ;
    if (this->dataPtr->jointVelCmd2 != 0) {
      targetVelFrontLeft = targetVelLeft * (
          sqrt(
            pow((this->dataPtr->axelDistance/tan(this->dataPtr->jointVelCmd2) - this->dataPtr->wheelDistance/2), 2) + pow(this->dataPtr->axelDistance, 2)
          ) / sqrt(pow((this->dataPtr->axelDistance/tan(this->dataPtr->jointVelCmd2) - this->dataPtr->wheelDistance/2), 2))
        );
      targetVelFrontRight = targetVelRight * (
          sqrt(
            pow((this->dataPtr->axelDistance/tan(this->dataPtr->jointVelCmd2) + this->dataPtr->wheelDistance/2), 2) + pow(this->dataPtr->axelDistance, 2)
          ) / sqrt(pow((this->dataPtr->axelDistance/tan(this->dataPtr->jointVelCmd2) + this->dataPtr->wheelDistance/2), 2))
        );
    } else {
      targetVelFrontLeft = this->dataPtr->jointVelCmd ;
      targetVelFrontRight = this->dataPtr->jointVelCmd ;
    }
    targetVel = this->dataPtr->jointVelCmd;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex2);
    position = this->dataPtr->jointVelCmd2;
  }

  {
    speedValue = 1/jointVelComp->Data().at(0);
  }

  // Force mode.
  if (this->dataPtr->useForceCommands)
  {

  }
  // Velocity mode.
  else
  {
    // Update joint velocity
    auto vel =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->jointEntity);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          this->dataPtr->jointEntity,
          components::JointVelocityCmd({targetVelLeft}));
    }
    else
    {
      vel->Data()[0] = targetVelLeft;
    }

    // Update joint velocity
    auto vel3 =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->jointEntity3);

    if (vel3 == nullptr)
    {
      _ecm.CreateComponent(
          this->dataPtr->jointEntity3,
          components::JointVelocityCmd({targetVelRight}));
    }
    else
    {
      vel3->Data()[0] = targetVelRight;
    }

     // Update joint velocity
    auto velLeftFront =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->frontWheelLeftJointEntity);

    if (velLeftFront == nullptr)
    {
      _ecm.CreateComponent(
          this->dataPtr->frontWheelLeftJointEntity,
          components::JointVelocityCmd({targetVelFrontLeft}));
    }
    else
    {
      velLeftFront->Data()[0] = targetVelFrontLeft;
    }

     // Update joint velocity
    auto velRightFront =
      _ecm.Component<components::JointVelocityCmd>(this->dataPtr->frontWheelRightJointEntity);

    if (velRightFront == nullptr)
    {
      _ecm.CreateComponent(
          this->dataPtr->frontWheelRightJointEntity,
          components::JointVelocityCmd({targetVelFrontRight}));
    }
    else
    {
      velRightFront->Data()[0] = targetVelFrontRight;
    }

    // Update force command.
  double error;
  double error2;
  double error_inner;
  double error2_inner;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex2);
    error = jointPosComp->Data().at(this->dataPtr->jointIndex) -
            atan((2.0 * this->dataPtr->axelDistance * sin(this->dataPtr->jointVelCmd2))/(2.0 * this->dataPtr->axelDistance * cos(this->dataPtr->jointVelCmd2) 
            + this->dataPtr->wheelDistance* sin(this->dataPtr->jointVelCmd2)));
  }
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex4);
    error2 = jointPosComp2->Data().at(this->dataPtr->jointIndex) - 
            atan((2.0 * this->dataPtr->axelDistance * sin(this->dataPtr->jointVelCmd2))/(2.0 * this->dataPtr->axelDistance * cos(this->dataPtr->jointVelCmd2) 
            - this->dataPtr->wheelDistance* sin(this->dataPtr->jointVelCmd2)));
  }

  
  double force = this->dataPtr->posPid.Update(error, _info.dt);
  double force2 = this->dataPtr->posPid2.Update(error2, _info.dt);

  error_inner  = jointPosComp->Data().at(this->dataPtr->jointIndex) - force;
  error2_inner = jointPosComp2->Data().at(this->dataPtr->jointIndex) - force2;

  double force_inner  = this->dataPtr->posPidInner.Update(error_inner, _info.dt);
  double force2_inner = this->dataPtr->posPid2Inner.Update(error2_inner, _info.dt);

  // std::cout << "outer force 1: " << force << std::endl;
  // std::cout << "outer force 2: " << force2 << std::endl;
  // std::cout << "inner force 1: " << force_inner << std::endl;
  // std::cout << "inner force 2: " << force2_inner << std::endl;
  // std::cout << "error: " << error << std::endl;
  // std::cout << "error 2: " << error2 << std::endl;
  // std::cout << "error inner: " << error_inner << std::endl;
  // std::cout << "error inner 2: " << error2_inner << std::endl;
  // std::cout << "positon 1: " << jointPosComp->Data().at(this->dataPtr->jointIndex) << std::endl;
  // std::cout << "position 2: " << jointPosComp2->Data().at(this->dataPtr->jointIndex) << std::endl;

  auto forceComp =
      _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity2);

  auto forceComp2 =
      _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity4);

  if (forceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity2,
                         components::JointForceCmd({force_inner}));
  }
  else
  {
    forceComp->Data()[this->dataPtr->jointIndex] = force_inner;
  }

  if (forceComp2 == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity4,
                         components::JointForceCmd({force2_inner}));
  }
  else
  {
    forceComp2->Data()[this->dataPtr->jointIndex] = force2_inner;
  }
  }

  
};

////////// Post Update //////
void SampleSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointController::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (this->dataPtr->carType == "old") {
    ignition::msgs::Double hall;
    ignition::msgs::Double hall_8;

    ignition::msgs::Double hall_cnt;
    
    hall.set_data(speedValue);

    counter = counter + 1;
    hall_cnt.set_data(counter);

    this->dataPtr->pub_pose.Publish(hall);

    if (std::fmod(counter, 8.0) == 0) {
      hall_8.set_data(speedValue * 8);
      this->dataPtr->pub_hall_8.Publish(hall_8);
    }

    this->dataPtr->pub_hall_cnt.Publish(hall_cnt);
  } else {
    ignition::msgs::Double enc_data;
    ignition::msgs::Double enc_step;

    counter = counter + 1;
    enc_data.set_data(speedValue * this->dataPtr->wheelDiameter);
    enc_step.set_data(counter);

    this->dataPtr->pub_enc_data.Publish(enc_data);
    this->dataPtr->pub_step_data.Publish(enc_step);
  }
}
//! [implementSampleSystem]

//////////////////////////////////////////////////
void SampleSystemPrivate::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
    this->jointVelCmd = ((float)_msg.data() * (4 / 1500.0) / this->wheelDiameter) * 0.55;
}

//////////////////////////////////////////////////
void SampleSystemPrivate::OnCmdVel2(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex2);
  
  if (abs(_msg.data()) > 0 ) {
    this->jointVelCmd2 = 0.78535 * (float)_msg.data() / (float)800 * 0.45;
  } else {
    this->jointVelCmd2 = 0;
  }
}

// Include a line in your source file for each interface implemented.
IGNITION_ADD_PLUGIN(
    SampleSystem,
    ignition::gazebo::System,
    SampleSystem::ISystemConfigure,
    SampleSystem::ISystemPreUpdate,
    SampleSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SampleSystem,
                          "ignition::gazebo::systems::SampleSystem")