#ifndef TM_GAZEBO_PLUGINS_HPP
#define TM_GAZEBO_PLUGINS_HPP
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "tm_msgs/msg/robot_status.hpp"
#include "tm_msgs/msg/joint_trajectorys.hpp"


#include <iostream>
using namespace std::chrono_literals;
namespace gazebo_plugins
{
  enum class ErrorCodeMessage{
    noError,
    idNotCorrect,
    modeError
  };

  class TMGazeboPluginRosPrivate {
    private:
      
      void message_publish();
      std::vector<std::vector<double>> tajectoryVelocity;
      std::vector<std::vector<double>> tajectoryPosition;

      std::vector<gazebo::physics::JointPtr> gazeboJoint;
      float initial_joints_value;
      float counter;
      std::shared_ptr< rclcpp::Publisher <tm_msgs::msg::RobotStatus> > motorStatusPublish;
      
      int controlMode = 0;
      bool pointExecute;
      int currentCommanderId;
      int commandCounter;
      ErrorCodeMessage errorCode;
      
    public:
      TMGazeboPluginRosPrivate();
      gazebo::physics::ModelPtr model;//TODO: change to set, get
      gazebo_ros::Node::SharedPtr rosNode;
      rclcpp::Node::SharedPtr listenNode;

      const int jointPositionControl =0;
      const int jointVelocityControl =1;      
      const unsigned int jointNumber=6;

      void initial_modle_pose() ;
      void set_model_joint() ;
      void create_topic();
      void set_command_to_gazebo() ;
      void set_command_to_gazebo_test();

      void listen_thread(rclcpp::Node::SharedPtr node);
      void create_listen_command_topic();

      
    
  };

  class TMGazeboPluginRos : public gazebo::ModelPlugin
  {
  public:
    /// Constructor
    TMGazeboPluginRos();

    /// Destructor
    ~TMGazeboPluginRos();

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
    
    std::unique_ptr<TMGazeboPluginRosPrivate> robot_simulator;
  };
}
#endif
