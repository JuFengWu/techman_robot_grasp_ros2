#pragma once
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class MoveitBridge {
private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr< rclcpp::Publisher <sensor_msgs::msg::JointState> > jointStatesChatter;
  std::shared_ptr< rclcpp::Publisher <sensor_msgs::msg::JointState> > jointTargetChatter;
  std::shared_ptr< rclcpp::Publisher <geometry_msgs::msg::Pose> > cartesianChatter;
  bool isGetResult;
  geometry_msgs::msg::Pose current_position;
  trajectory_msgs::msg::JointTrajectory trajectory;
  bool isListen;

  void listen_thread(rclcpp::Node::SharedPtr node);
  void wait_connect_success(std::shared_ptr< rclcpp::Publisher <sensor_msgs::msg::JointState> > publisher);
  void wait_connect_success(std::shared_ptr< rclcpp::Publisher <geometry_msgs::msg::Pose> > publisher);
public:
  MoveitBridge();
  ~MoveitBridge();
  void set_current_joint(std::vector<double> jointPosition);
  trajectory_msgs::msg::JointTrajectory get_trajectories(std::vector<double> jointTarget);
  trajectory_msgs::msg::JointTrajectory get_trajectories(geometry_msgs::msg::Pose eeTarget);
  geometry_msgs::msg::Pose get_current_position();
};