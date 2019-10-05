#include<vector>
#include <rclcpp/rclcpp.hpp>
#include "tm_msgs/msg/robot_status.hpp"

#include <iostream>
namespace tm_driver{
  class ListenRobotSatus{
  private:
    bool isListen;
    float loopRate;
    std::vector<double> jointPostion;
    std::vector<double> jointVelocity;
    std::vector<double> jointTorque;
    void listen_thread(rclcpp::Node::SharedPtr node);
    void set_loop_rate(float loopRate);
    
  public:
    ListenRobotSatus(rclcpp::Node::SharedPtr node,float loopRate);
    ~ListenRobotSatus();
    const int jointNumber =6;
    void set_loop_rate(int loopRate);
    std::vector<double> get_joint_position();
    std::vector<double> get_joint_velocity();
    std::vector<double> get_joint_torque();
  
  };
}