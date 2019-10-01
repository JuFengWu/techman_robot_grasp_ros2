

#include"../include/tm_gazebo_plugin/robot_simulator.hpp"
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
    RobotSimulatorInterface *robot_simulator;
  };
}
