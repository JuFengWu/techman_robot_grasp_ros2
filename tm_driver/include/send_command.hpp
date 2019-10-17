#include<vector>
#include <iostream>
#include "tm_msgs/msg/joint_trajectorys.hpp"
#include "tm_msgs/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
namespace tm_driver{
  
  class SendCommand{
  private:
    std::shared_ptr< rclcpp::Publisher <tm_msgs::msg::JointTrajectorys> > jointCommabdPublish;
    rclcpp::Node::SharedPtr node;
    int jointNumber =6;
    const int myCommanderId = 9988;
    int currentCommanderId = 0;
    bool isProcessCmd = false;
    void listen_thread();
    int commandCounter = 0;
    
  public:
    SendCommand(rclcpp::Node::SharedPtr node);
    void send_joint_position(std::vector<std::vector<double>> jointPosition);
    void send_joint_velocity(std::vector<std::vector<double>> jointVelocity);
  };
}

