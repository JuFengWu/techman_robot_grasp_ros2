#include"../include/robot_simulator.h"

void TmRobotSimulator::set_model_joint(std::vector<gazebo::physics::JointPtr> gazeboJoint){
  this->gazeboJoint.resize(this->jointNumber);
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

  for(int i=0;i<this->jointNumber;i++){
    motor_status_msg.current_joint_position.push_back(gazeboJoint[i]->Position(0));
    motor_status_msg.current_joint_velocity.push_back(gazeboJoint[i]->GetVelocity(0));
    motor_status_msg.current_joint_force.push_back(gazeboJoint[i]->GetForce(0));
  } 

  motorStatusPublish->publish(motor_status_msg);
}
void TmRobotSimulator::create_topic(){
  rosNode = rclcpp::Node::make_shared("techman_joint_states_publish");

  motorStatusPublish = rosNode->create_publisher<tm_msgs::msg::RobotStatus>("tm_motor_state",rclcpp::SensorDataQoS());
  
  rosNode->create_wall_timer(50ms,std::bind(&TmRobotSimulator::message_publish, this.get()));
}


void TmRobotSimulator::execute_joint_move(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle){
  const auto goal = goalHandle->get_goal();
  
  std::vector<int> tajectorySize;
  if( goal->trajectory.points.size() == 0){
        std::cout<<"points are empty!"<<std::endl;
        return;
  }
  if(this->jointPositionControl == joint_control_mode){ 
    for(unsigned int i = 0; i < this->jointNumber; i++){
      for(unsigned int j = 0; j < goal->joint_trajectory.points[i].positions.size(); j++){
        tajectoryPosition[i][j] =  goal->joint_trajectory.points[i].positions[j];
      }
      std::reverse(tajectoryPosition[i].begin(),tajectoryPosition[i].end())
      tajectorySize.push_back(goal->joint_trajectory.points[i].positions.size());
    }
    controlMode =this->jointPositionControl;
  }
  else if (this->jointVelocityControl == joint_control_mode)
  {
    for(unsigned int i = 0; i < this->jointNumber; i++){
      for(unsigned int j = 0; j < goal->joint_trajectory.points[i].velocities.size(); j++){
        tajectoryVelocity[i][j] =  goal->joint_trajectory.points[i].velocities[j];
      }
      tajectorySize.push_back(goal->joint_trajectory.points[i].velocities.size());
    }
    controlMode =this->jointVelocityControl;
  }
  else
  {
    std::cout<<"mode error!"<<std::endl;
    return;
  }
  
  pointExecute = true;
  int mostPoint = max_element(std::begin(tajectorySize), std::end(tajectorySize));
  auto feedback = std::make_shared<tm_msgs::action::JointTrajectory::Feedback>();
  rclcpp::Rate loop_rate(1);

  while(pointExecute){
    std::vector<int> currentTajectorySize;
    for(unsigned int i = 0; i < this->jointNumber; i++){
      if(this->jointPositionControl == joint_control_mode){
        currentTajectorySize.push_back(tajectoryPosition[i].size());
      }
      else{
        currentTajectorySize.push_back(tajectoryVelocity[i].size());
      }
      
    }
    int currentMostPoint = max_element(std::begin(currentTajectorySize), std::end(currentTajectorySize));
    double executePersent = 1-(currentMostPoint/mostPoint);
    feedback->process_persent = executePersent;
    loop_rate.sleep();
  }
  auto result = std::make_shared<tm_msgs::action::JointTrajectory::Result>();
  result->error_code = 0;
  result->is_success = 1;
  goalHandle->succeed(result_response);
  std::cout<<"success!"<<std::endl;
  
}
void TmRobotSimulator::handle_tm_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const tm_msgs::action::JointTrajectory::Goal> goal){
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request");// TODO: rclcpp::get_logger("server") may have problem
  (void)uuid;
  if (goal->robot_id <0) {
    std::cout<<"robot_id should bigger than 0"<<std::endl;
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}
void TmRobotSimulator::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle){
  std::cout<<"Got request to cancel goal"<<std::endl;
  (void)goalHandle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
void TmRobotSimulator::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle){
  std::cout<<"Accept action!"<<std::endl;
  std::thread(&TmRobotSimulator::execute_joint_move, this, goalHandle).detach();
}
void TmRobotSimulator::create_command_action(){
  auto action_server = rclcpp_action::create_server<tm_msgs::action::JointTrajectory>(
    rosNode,
    "techman_action_server",
    handle_tm_action_goal,
    handle_cancel,
    handle_accepted);
}
void TmRobotSimulator::set_command_to_gazebo(std::vector<gazebo::physics::JointPtr> gazeboJoint){
  if(this->jointPositionControl == joint_control_mode){
    for(unsigned int i=0 ; i < this->jointNumber; i++){
      if(tajectoryPosition[i].empty()){
        jointValue[i] = gazeboJoint[i]->Position[i]
      }
      else{
        jointValue[i] = tajectoryPosition[i].pop_back()
      }
      gazeboJoint[i]->SetPosition(0, jointValue[i], false);// TODO: check update time
          
    }
  }
  else // velocity mode
  {
    for(unsigned int i=0 ; i < this->jointNumber; i++){
      if(tajectoryVelocity[i].empty()){
        jointValue[i] = gazeboJoint[i]->GetVelocity(i)
      }
      else{
        jointValue[i] = tajectoryVelocity[i].pop_back()
      }
      gazeboJoint[i]->SetVelocity(0, jointValue[i], false);
      
    }
  }
  
}

void TmRobotSimulator::initial_modle_pose(){
  for(unsigned i = 0; i < this->jointNumber; i++)
  {
     gazeboJoint[i]->SetPosition(0, initial_joints_value, false);
     lastJointValue[i] = initial_joints_value;
  }
}
