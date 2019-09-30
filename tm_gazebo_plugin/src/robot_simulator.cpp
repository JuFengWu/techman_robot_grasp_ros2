#include"../include/robot_simulator.h"

void TmRobotSimulator::set_model_joint(std::vector<gazebo::physics::JointPtr> gazeboJoint){
  this->gazeboJoint.resize(this->joint_number);
  this->gazeboJoint[0] = this->model->GetJoint("shoulder_1_joint");
  this->gazeboJoint[1] = this->model->GetJoint("shoulder_2_joint");
  this->gazeboJoint[2] = this->model->GetJoint("elbow_1_joint");
  this->gazeboJoint[3] = this->model->GetJoint("wrist_1_joint");
  this->gazeboJoint[4] = this->model->GetJoint("wrist_2_joint");
  this->gazeboJoint[5] = this->model->GetJoint("wrist_3_joint");
  
  initial_joints_value = 0.0f;// TODO: move to construct
  initial_modle_pose();

}
void TmRobotSimulator::message_publish(){
  tm_msgs::msg::RobotStatus motor_status_msg;

  for(int i=0;i<this->joint_number;i++){
    motor_status_msg.current_joint_position.push_back(gazeboJoint[i]->Position(0));
    motor_status_msg.current_joint_velocity.push_back(gazeboJoint[i]->GetVelocity(0));
    motor_status_msg.current_joint_force.push_back(gazeboJoint[i]->GetForce(0));
  } 

  motor_status_pub->publish(motor_status_msg);
}
void TmRobotSimulator::create_topic(){
  joint_status_publish_node = rclcpp::Node::make_shared("techman_joint_states_publish");

  motor_status_pub = joint_status_publish_node->create_publisher<tm_msgs::msg::RobotStatus>("tm_motor_state",rclcpp::SensorDataQoS());
  
  joint_status_publish_node->create_wall_timer(50ms,std::bind(&TmRobotSimulator::message_publish, this.get()));
}


void TmRobotSimulator::execute_joint_move(const std::shared_ptr<rclcpp_action::ServerGoalHandle<JointTrajectory>> goal_handle){
  const auto goal = goal_handle->get_goal();
  if( goal->trajectory.points.size() == 0){
        std::cout<<"points are empty!"<<std::endl;
        return;
  }
  for(unsigned int i = 0; i < goal->trajectory.points.size(); i++){
    tajectory_vel[i] =  goal->trajectory.points[i].velocities[0];
    tajectory_position[i] =  goal->trajectory.points[i].positions[0];
  }
}
void TmRobotSimulator::handle_tm_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointTrajectory::Goal> goal){
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request");// TODO: rclcpp::get_logger("server") may have problem
  (void)uuid;
  if (goal->robot_id <0) {
    std::cout<<"robot_id should bigger than 0"<<std::endl;
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}
void TmRobotSimulator::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<JointTrajectory>> goal_handle){
  std::cout<<"Got request to cancel goal"<<std::endl;
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
void TmRobotSimulator::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<JointTrajectory>> goal_handle){
  std::cout<<"Accept action!"<<std::endl;
  std::thread(&TmRobotSimulator::execute_joint_move, this, goal_handle).detach();
}
void TmRobotSimulator::create_command_action(){

}
void TmRobotSimulator::set_command_to_gazebo(std::vector<gazebo::physics::JointPtr> gazeboJoint){
  
}
void TmRobotSimulator::get_current_state_to_publish(std::vector<gazebo::physics::JointPtr> gazeboJoint){

}

void TmRobotSimulator::initial_modle_pose(){
  gazeboJoint[0]->SetPosition(0, initial_joints_value, false);
  gazeboJoint[1]->SetPosition(0, initial_joints_value, false);
  gazeboJoint[2]->SetPosition(0, initial_joints_value, false);
  gazeboJoint[3]->SetPosition(0, initial_joints_value, false);
  gazeboJoint[4]->SetPosition(0, initial_joints_value, false);
  gazeboJoint[5]->SetPosition(0, initial_joints_value, false);
}
