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
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>

#include "tm_msgs/msg/robot_status.hpp"
#include "tm_msgs/action/joint_trajectory.hpp"


#include <iostream>
using namespace std::chrono_literals;
namespace gazebo_plugins
{
  class RobotSimulatorInterface{
    public:
      /// Pointer to model.
      gazebo::physics::ModelPtr model;//TODO: change to set, get
      gazebo_ros::Node::SharedPtr rosNode;

      const int jointPositionControl =0;
      const int jointVelocityControl =1;
      int controlMode = 0;
      virtual void set_model_joint();
      virtual void create_topic();
      virtual void create_command_action();
      virtual void set_command_to_gazebo();
    protected:
      std::vector<gazebo::physics::JointPtr> gazeboJoint;
      float initial_joints_value;
      
      std::shared_ptr< rclcpp::Publisher <tm_msgs::msg::RobotStatus> > motorStatusPublish;
  };

  class TmRobotSimulator : public RobotSimulatorInterface{
    private:
      unsigned int jointNumber=6;
      void message_publish();
      std::vector<std::vector<double>> tajectoryVelocity;
      std::vector<std::vector<double>> tajectoryPosition;
      void initial_modle_pose();
    public:
      void set_model_joint() override;
      void create_topic() override;
      void create_command_action() override;
      void set_command_to_gazebo() override;
      rclcpp_action::GoalResponse handle_tm_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const tm_msgs::action::JointTrajectory::Goal> goal);
      rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle);
      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle);
      void execute_joint_move(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle);
      double jointValue[6];
      int joint_control_mode;
      bool pointExecute;
    
  };
}
#endif