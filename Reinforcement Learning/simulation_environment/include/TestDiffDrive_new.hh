#ifndef TEST_DIFF_DRIVE_HH
#define TEST_DIFF_DRIVE_HH

#include <memory>
#include <gz/sim7/System.hh>

// 注意：下面命名空间改成 gz::sim，而非 ignition::gazebo
namespace gz
{
namespace sim
{
// 为了适配 Garden，这里有个 inline 命名空间 GZ_SIM_VERSION_NAMESPACE，也就是 v7
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  // 先前你写的 SampleSystem 也可改名 TestDiffDrive，以示差动驱动插件
  
  // Forward declare
  class TestDiffDrivePrivate;

  /// \brief A sample system plugin that demonstrates usage in Garden
  class TestDiffDrive
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate
  {
    // Constructor
    public: TestDiffDrive();

    // Destructor
    public: ~TestDiffDrive() override = default;

    // Configure system
    public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

    // PreUpdate
    public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    // PostUpdate
    public: void PostUpdate(
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm) override;

    // Private data pointer
    private: std::unique_ptr<TestDiffDrivePrivate> dataPtr;
  };

}
}
}

#endif
