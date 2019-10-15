#include<vector>
#include <rclcpp/rclcpp.hpp>
#include "tm_msgs/msg/robot_status.hpp"

#include <iostream>
namespace tm_driver{
  class ListenRobotSatus{
  private:
    bool isListen;
    std::vector<double> jointPostion;
    std::vector<double> jointVelocity;
    std::vector<double> jointTorque;

    bool isProcessCmd;
    int commanderId;
    int errorCode;
    void listen_thread(rclcpp::Node::SharedPtr node);
    
  public:
    ListenRobotSatus(rclcpp::Node::SharedPtr node);
    ~ListenRobotSatus();
    const int jointNumber =6;
    std::vector<double> get_joint_position();
    std::vector<double> get_joint_velocity();
    std::vector<double> get_joint_torque();
  
  };
}