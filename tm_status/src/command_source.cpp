#include "../include/command_source.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/clock.hpp"
#include <cmath>
#include <iostream>


int TestMoveCommand::get_loop_time(){
    return 50;//ms
}

void TestMoveCommand::get_outside_command(){
    return;
}

void TestMoveCommand::run_command(sensor_msgs::msg::JointState& msg,rclcpp::Clock::SharedPtr clock){
    counter += 0.005;
    double joint_value = std::sin(counter);
    for (size_t i = 0; i < 6; ++i) {
      
      msg.position[i] = joint_value;
    }
    msg.header.stamp = clock->now();
    
}