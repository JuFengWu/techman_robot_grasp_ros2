#include "../include/model.h"
#include <iostream>
void ModelInterface::set_model_initial_pose(sensor_msgs::msg::JointState& msg,double allInitialPose =0){
  for(unsigned int i=0;i<msg.name.size();i++){
      msg.position.push_back(allInitialPose);
  }
  std::cout<<"msg.name.size() is"<<msg.name.size()<<std::endl;
}

void TmGraspModel::set_model_name(sensor_msgs::msg::JointState& msg){
  msg.name.push_back("shoulder_1_joint");
  msg.name.push_back("shoulder_2_joint");
  msg.name.push_back("elbow_1_joint");
  msg.name.push_back("wrist_1_joint");
  msg.name.push_back("wrist_2_joint");
  msg.name.push_back("wrist_3_joint");

  msg.name.push_back("tip_fixed_joint");
  msg.name.push_back("connect_joint");
  msg.name.push_back("finger_joint");

  msg.name.push_back("left_outer_finger_joint");
  msg.name.push_back("left_inner_knuckle_joint");
  msg.name.push_back("left_inner_finger_joint");
  msg.name.push_back("left_inner_finger_pad_joint");
  msg.name.push_back("right_outer_knuckle_joint");
  msg.name.push_back("right_outer_finger_joint");
  msg.name.push_back("right_inner_knuckle_joint");
  msg.name.push_back("right_inner_finger_joint");
  msg.name.push_back("right_inner_finger_pad_joint");
}
