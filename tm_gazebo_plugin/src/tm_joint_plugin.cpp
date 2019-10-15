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
      motorStatusMsg->error_code = static_cast<int> (errorCode);
      motorStatusMsg->is_process_cmd = pointExecute;
      motorStatusMsg->commander_id = currentCommanderId;
      
      motorStatusPublish->publish(*motorStatusMsg);
      loop_rate.sleep();
    }
    
  }
  void TMGazeboPluginRosPrivate::create_topic(){
    motorStatusPublish = rosNode->create_publisher<tm_msgs::msg::RobotStatus>("tm_motor_states",rclcpp::QoS(100));

    std::thread(&TMGazeboPluginRosPrivate::message_publish,this).detach();
  }

  void TMGazeboPluginRosPrivate::listen_thread(rclcpp::Node::SharedPtr node){
    
    auto callBack =[this](const tm_msgs::msg::JointTrajectorys::SharedPtr jointTrajectorys) -> void
        {
          std::cout<<"hear a command!!"<<std::endl;
          int robotID = jointTrajectorys->robot_id;
          std::cout<<"robotID is "<<robotID<<std::endl;
          if(jointTrajectorys->robot_id<0){
            std::cout<<"id is not correct"<<std::endl;
            errorCode = ErrorCodeMessage::idNotCorrect;
          }
          currentCommanderId = jointTrajectorys->commander_id;
          controlMode = jointTrajectorys->joint_control_mode;
          if(this->jointPositionControl == controlMode){ 
            for(unsigned int i = 0; i < this->jointNumber; i++){
              for(unsigned int j = 0; j < jointTrajectorys->joint_trajectory.points[i].positions.size(); j++){
                tajectoryPosition[i].push_back( jointTrajectorys->joint_trajectory.points[i].positions[j]);
              }
              std::reverse(tajectoryPosition[i].begin(),tajectoryPosition[i].end());
            
            }
            
          }
          else if (this->jointVelocityControl == controlMode)
          {
            for(unsigned int i = 0; i < this->jointNumber; i++){
              for(unsigned int j = 0; j < jointTrajectorys->joint_trajectory.points[i].velocities.size(); j++){
                tajectoryVelocity[i].push_back( jointTrajectorys->joint_trajectory.points[i].velocities[j]);
              }
              std::reverse(tajectoryVelocity[i].begin(),tajectoryVelocity[i].end());
            }
            
          }
          else
          {
            std::cout<<"mode error!"<<std::endl;
            errorCode = ErrorCodeMessage::modeError;
            return;
          }
          pointExecute = true;
        };
    auto subscription = node->create_subscription<tm_msgs::msg::JointTrajectorys>("joint_trajectory_msgs", rclcpp::QoS(100), callBack);

    std::cout<<"in thread!!"<<std::endl;

    rclcpp::spin(node);

    std::cout<<"after thread spin node!!"<<std::endl;
    
    rclcpp::shutdown();
  }

  void TMGazeboPluginRosPrivate::create_listen_command_topic(){
      
      std::thread(&TMGazeboPluginRosPrivate::listen_thread,this,listenNode).detach();
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
    else if (this->jointVelocityControl == controlMode)
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
    else{
      std::cout<<"control mode error!!"<<std::endl;
    }
    if(!isMove){
      pointExecute = false;
      currentCommanderId = 0;
    }
    else{
      errorCode = ErrorCodeMessage::noError;
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
        this->robot_simulator->listenNode = rclcpp::Node::make_shared("ListenCommand");        
        
        this->robot_simulator->set_model_joint();
        
        this->robot_simulator->create_topic();
        
        this->robot_simulator->create_listen_command_topic();

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
