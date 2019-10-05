#include<vector>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "tm_msgs/action/joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
namespace tm_driver{
  
  using JointTrajectory = tm_msgs::action::JointTrajectory;
  
  class SendCommand{
  private:
    
    rclcpp_action::Client<JointTrajectory>::SharedPtr actionClient;
    JointTrajectory::Goal goalMsg;
    
    rclcpp::Node::SharedPtr node;
    bool is_success;
    int error_code;

    //static void feedback_callback(SendCommand *sendCommand,,rclcpp_action::ClientGoalHandle<JointTrajectory>::SharedPtr,const std::shared_ptr<const JointTrajectory::Feedback> feedback);
    int check_result_correct(rclcpp_action::ClientGoalHandle<JointTrajectory>::WrappedResult wrappedResult);
    int send_goal();
  public:
    double processPersent;
    SendCommand(rclcpp::Node::SharedPtr node);
    int send_joint_position(std::vector<std::vector<double>> jointPosition);
    int send_joint_velocity(std::vector<std::vector<double>> jointVelocity);
    double check_process();
    int get_error_code();
  };
}

