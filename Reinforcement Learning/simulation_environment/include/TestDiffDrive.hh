#include <ignition/gazebo/System.hh>

#include <cmath>

namespace ignition
{
namespace gazebo
{
  class SampleSystemPrivate;

  class SampleSystem:
        public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    public: SampleSystem();
    public: ~SampleSystem() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;
    
    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
      
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<SampleSystemPrivate> dataPtr;

    private: double speedValue;
    private: double counter;
  };
}
}
