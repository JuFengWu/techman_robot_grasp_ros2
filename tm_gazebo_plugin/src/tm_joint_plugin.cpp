#include"../include/tm_gazebo_plugin/tm_joint_plugin.hpp"
namespace gazebo_plugins
{

  TMGazeboPluginRosPrivate::TMGazeboPluginRosPrivate(){
    tajectoryPosition.resize(jointNumber);
    tajectoryVelocity.resize(jointNumber);
  }
  void TMGazeboPluginRosPrivate::set_model_joint(){
    
    this->gazeboJoint.resize(this->jointNumber);
    this->gazeboJoint[0] = this->model->GetJoint("shoulder_1_joint");
    this->gazeboJoint[1] = this->model->GetJoint("shoulder_2_joint");
    this->gazeboJoint[2] = this->model->GetJoint("elbow_1_joint");
    this->gazeboJoint[3] = this->model->GetJoint("wrist_1_joint");
    this->gazeboJoint[4] = this->model->GetJoint("wrist_2_joint");
    this->gazeboJoint[5] = this->model->GetJoint("wrist_3_joint");
    
    initial_joints_value = 0.0f;// TODO: move to construct
    this->initial_modle_pose();

  }
  void TMGazeboPluginRosPrivate::message_publish(){
    //tm_msgs::msg::RobotStatus motor_status_msg;

    auto motorStatusMsg = std::make_shared<tm_msgs::msg::RobotStatus>();
    rclcpp::WallRate loop_rate(1s);
    motorStatusMsg->current_joint_position.resize(jointNumber);
    motorStatusMsg->current_joint_velocity.resize(jointNumber);
    motorStatusMsg->current_joint_force.resize(jointNumber);
    int counter =0;
    while (rclcpp::ok())
    {
      motorStatusMsg->current_joint_position.clear();
      motorStatusMsg->current_joint_velocity.clear();
      motorStatusMsg->current_joint_force.clear();

      for(unsigned int i=0;i<this->jointNumber;i++){    
        motorStatusMsg->current_joint_position.push_back(gazeboJoint[i]->Position(0)); 
        motorStatusMsg->current_joint_velocity.push_back(gazeboJoint[i]->GetVelocity(0));
        motorStatusMsg->current_joint_force.push_back(gazeboJoint[i]->GetForce(0));
      } 
      //std::cout<<"send msg "<<counter<<std::endl;
      counter++;
      motorStatusPublish->publish(*motorStatusMsg);
      loop_rate.sleep();
    }
    
  }
  void TMGazeboPluginRosPrivate::create_topic(){
    motorStatusPublish = rosNode->create_publisher<tm_msgs::msg::RobotStatus>("tm_motor_states",rclcpp::QoS(100));

    std::thread(&TMGazeboPluginRosPrivate::message_publish,this).detach();
  }

  void TMGazeboPluginRosPrivate::execute_joint_move(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle){
    const auto goal = goalHandle->get_goal();
    std::cout<<"get_goal"<<std::endl;
    
    std::vector<int> tajectorySize;
    if( goal->joint_trajectory.points.size() == 0){
          std::cout<<"points are empty!"<<std::endl;
          return;
    }
    std::cout<<"after check something"<<std::endl;
    if(this->jointPositionControl == controlMode){ 
      for(unsigned int i = 0; i < this->jointNumber; i++){
        std::cout<<"aaa"<<std::endl;
        for(unsigned int j = 0; j < goal->joint_trajectory.points[i].positions.size(); j++){
          tajectoryPosition[i].push_back( goal->joint_trajectory.points[i].positions[j]);
        }
        std::reverse(tajectoryPosition[i].begin(),tajectoryPosition[i].end());
        tajectorySize.push_back(goal->joint_trajectory.points[i].positions.size());
      }
      
    }
    else if (this->jointVelocityControl == controlMode)
    {
      for(unsigned int i = 0; i < this->jointNumber; i++){
        for(unsigned int j = 0; j < goal->joint_trajectory.points[i].velocities.size(); j++){
          tajectoryVelocity[i].push_back( goal->joint_trajectory.points[i].velocities[j]);
        }
        tajectorySize.push_back(goal->joint_trajectory.points[i].velocities.size());
      }
      
    }
    else
    {
      std::cout<<"mode error!"<<std::endl;
      return;
    }
    std::cout<<"get points"<<std::endl;
    pointExecute = true;
    int mostPoint = *std::max_element(std::begin(tajectorySize), std::end(tajectorySize));
    auto feedback = std::make_shared<tm_msgs::action::JointTrajectory::Feedback>();
    auto resultResponse = std::make_shared<tm_msgs::action::JointTrajectory::Result>();
    rclcpp::Rate loop_rate(1s);

    while(pointExecute){
      std::vector<int> currentTajectorySize;
      for(unsigned int i = 0; i < this->jointNumber; i++){
        if(this->jointPositionControl == controlMode){
          currentTajectorySize.push_back(tajectoryPosition[i].size());
        }
        else{
          currentTajectorySize.push_back(tajectoryVelocity[i].size());
        }
        
      }
      // Check if there is a cancel request
      if (goalHandle->is_canceling()) {
        resultResponse->error_code = 0;
        resultResponse->is_success = 0;
        goalHandle->canceled(resultResponse);
        std::cout<<"Goal Canceled"<<std::endl;
        return;
      }
      int currentMostPoint = *std::max_element(std::begin(currentTajectorySize), std::end(currentTajectorySize));
      double executePersent = 1-(currentMostPoint/(double)mostPoint);
      feedback->process_persent = executePersent;
      goalHandle->publish_feedback(feedback);
      
      std::cout<<"executePersent is "<<executePersent<<std::endl;
      loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      resultResponse->error_code = 0;
      resultResponse->is_success = 1;

      goalHandle->succeed(resultResponse);
      
      std::cout<<"success!"<<std::endl;
    }
    
  }
  rclcpp_action::GoalResponse TMGazeboPluginRosPrivate::handle_tm_action_goal
    (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const tm_msgs::action::JointTrajectory::Goal> goal){
    RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request");// TODO: rclcpp::get_logger("server") may have problem
    (void)uuid;
    if (goal->robot_id <0) {
      std::cout<<"robot_id should bigger than 0"<<std::endl;
      return rclcpp_action::GoalResponse::REJECT;
    }
    controlMode = goal->joint_control_mode;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

  }
  rclcpp_action::CancelResponse TMGazeboPluginRosPrivate::handle_cancel
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle){
    std::cout<<"Got request to cancel goal"<<std::endl;
    (void)goalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void TMGazeboPluginRosPrivate::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tm_msgs::action::JointTrajectory>> goalHandle){
    std::cout<<"Accept action!"<<std::endl;
    std::thread(&TMGazeboPluginRosPrivate::execute_joint_move, this, goalHandle).detach();
  }
  void TMGazeboPluginRosPrivate::create_command_action(){
    auto action_server = rclcpp_action::create_server<tm_msgs::action::JointTrajectory>(
      rosNode,
      "techman_action_server",
      std::bind(&TMGazeboPluginRosPrivate::handle_tm_action_goal,this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TMGazeboPluginRosPrivate::handle_cancel,this,std::placeholders::_1),
      std::bind(&TMGazeboPluginRosPrivate::handle_accepted,this,std::placeholders::_1));
  }
  void TMGazeboPluginRosPrivate::set_command_to_gazebo_test(){

  this->counter += 0.00005;
    float joint_value = std::sin(this->counter);
    //std::cout<<"tm joint plugin joint_value is "<<joint_value<<std::endl;
    this->gazeboJoint[0]->SetPosition(0, joint_value, false);
    this->gazeboJoint[1]->SetPosition(0, joint_value, false);
    this->gazeboJoint[2]->SetPosition(0, joint_value, false);
    this->gazeboJoint[3]->SetPosition(0, joint_value, false);
    this->gazeboJoint[4]->SetPosition(0, joint_value, false);
    this->gazeboJoint[5]->SetPosition(0, joint_value, false);
  }
  void TMGazeboPluginRosPrivate::set_command_to_gazebo(){
    bool isMove = false;
    double jointValue[6];
    if(this->jointPositionControl == controlMode){
      
      for(unsigned int i=0 ; i < this->jointNumber; i++){    
        if(tajectoryPosition[i].empty()){  
          jointValue[i] = gazeboJoint[i]->Position(i);
        }
        else{
          jointValue[i] = tajectoryPosition[i].back();
          tajectoryPosition[i].pop_back(); 
          isMove = true;
        }
        //std::cout<<"position jointValue[i] is"<<jointValue[i];
        gazeboJoint[i]->SetPosition(0, jointValue[i], false);// TODO: check update time
            
      }
     // std::cout<<" "<<std::endl;
    }
    else // velocity mode
    {
      for(unsigned int i=0 ; i < this->jointNumber; i++){
        if(tajectoryVelocity[i].empty()){
          jointValue[i] = gazeboJoint[i]->GetVelocity(i);
        }
        else{
          jointValue[i] = tajectoryVelocity[i].back();
          tajectoryVelocity[i].pop_back();
          isMove = true;
        }
        //std::cout<<"vleocity jointValue[i] is"<<jointValue[i];
        gazeboJoint[i]->SetVelocity(0, jointValue[i]);
      }
      //std::cout<<" "<<std::endl;
    }
    if(!isMove){
      pointExecute = false;
    }
  }

  void TMGazeboPluginRosPrivate::initial_modle_pose(){
    for(unsigned i = 0; i < this->jointNumber; i++)
    {
      gazeboJoint[i]->SetPosition(0, initial_joints_value, false);
    }
  }

  TMGazeboPluginRos::TMGazeboPluginRos()
  : robot_simulator(std::make_unique<TMGazeboPluginRosPrivate>())
  {
  }

  TMGazeboPluginRos::~TMGazeboPluginRos()
  {
  }

  void TMGazeboPluginRos::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf){

        
        this->robot_simulator = std::make_unique<TMGazeboPluginRosPrivate>();
        
        this->robot_simulator->model = model;
        this->robot_simulator->rosNode = gazebo_ros::Node::Get(_sdf);
        
        
        
        this->robot_simulator->set_model_joint();
        
        this->robot_simulator->create_topic();
        
        this->robot_simulator->create_command_action();
        

        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&TMGazeboPluginRos::OnUpdate, this));
        std::cout<<"end initial"<<std::endl;
    }

    void TMGazeboPluginRos::OnUpdate(){
        this->robot_simulator->set_command_to_gazebo();
    }

    void TMGazeboPluginRos::Reset(){
        this->robot_simulator->initial_modle_pose();
    }
    GZ_REGISTER_MODEL_PLUGIN(TMGazeboPluginRos)
}
