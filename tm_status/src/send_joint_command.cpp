#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include "../include/command_source.h"
#include "../include/model.h"
#include "rclcpp/clock.hpp"

std::tuple< std::string, std::string> get_initial_parameter(){
  return std::make_tuple("tmGraspModel", "testMoveCommand");
}

ModelInterface* get_modle(std::string modleSting){
  if(modleSting == "tmGraspModel"){
    return new TmGraspModel();
  }
  throw "get_modle string error!";
}

std::unique_ptr<CommandSourceInterface> get_command(std::string commandString){
  if(commandString == "testMoveCommand"){
    //return new TestMoveCommand();
    return std::make_unique<TestMoveCommand>();
  }
  throw "get_command string error!";
}

rclcpp::Clock::SharedPtr set_time(rclcpp::Node::SharedPtr node){
  rclcpp::TimeSource timeSource(node);
  
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  
  timeSource.attachClock(clock);

  return clock;
}

void set_model(std::string modle_name, sensor_msgs::msg::JointState& msg){
  ModelInterface* modle = get_modle(modle_name);

  modle->set_model_name(msg);

  modle->set_model_initial_pose(msg,0.0);
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("techman_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);

  sensor_msgs::msg::JointState msg;

  try{
    auto parameters = get_initial_parameter();

    set_model(std::get<0>(parameters),msg);

    //CommandSourceInterface* command = get_command(std::get<1>(parameters));
    
    std::unique_ptr<CommandSourceInterface> command = get_command(std::get<1>(parameters));
    
    rclcpp::Clock::SharedPtr clock = set_time(node);

    rclcpp::WallRate loop_rate(command->get_loop_time());
    while (rclcpp::ok()) {
     
      command->run_command(msg,clock);
      
      joint_state_pub->publish(msg);
      
      rclcpp::spin_some(node);
      
      loop_rate.sleep();
    }
  }
  catch(const char * str){
    std::cout<<str<<std::endl;
    return 0;
  }
  rclcpp::shutdown();

  return 0;
}
