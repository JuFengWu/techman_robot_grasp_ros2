#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <iostream>

// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo_plugins
{
  class TMGazeboPluginRos : public gazebo::ModelPlugin
  {
  public:
    /// Constructor
    //TMGazeboPluginRos();

    /// Destructor
    //~TMGazeboPluginRos();

    void OnUpdate();

  protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;
  private: 
    gazebo::physics::ModelPtr _model;
    // Pointer to the update event connection
    gazebo::event::ConnectionPtr updateConnection;
    float counter;
    std::vector<gazebo::physics::JointPtr> _joints;
  };
}
