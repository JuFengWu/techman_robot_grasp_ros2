#include "../include/moveit_bridge.h"


void MoveitBridge::joint_trajectory_listen_thread(rclcpp::Node::SharedPtr node){
    
  auto callBack =[this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) -> void{
    std::cout<<"Hear trajectory"<<std::endl;
    this->trajectory.points = msg->points;
    this->isGetResult = true;    
  };
    
  auto jointTrajectorySub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>("tm_moveit_joint_trajectory", 10,callBack);

  rclcpp::spin(node);
  
  rclcpp::shutdown();
}

void MoveitBridge::current_position_listen_thread(rclcpp::Node::SharedPtr node){
  auto callBack =[this](const geometry_msgs::msg::Pose::SharedPtr msg) -> void{
    this->current_position = *msg;
  };
  auto currentPositionSub = node->create_subscription<geometry_msgs::msg::Pose>("tm_moveit_current_position",10,callBack);

  rclcpp::spin(node);
  
  rclcpp::shutdown();
}

void MoveitBridge::wait_connect_success(std::shared_ptr< rclcpp::Publisher <geometry_msgs::msg::Pose> > publisher){
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
  while(publisher->get_subscription_count()== 0){
    std::cout<<"wait connect success!"<<std::endl;
    loop_rate.sleep();
  }
}

void MoveitBridge::wait_connect_success(std::shared_ptr< rclcpp::Publisher <sensor_msgs::msg::JointState> > publisher){
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
  while(publisher->get_subscription_count()== 0){
    std::cout<<"wait connect success!"<<std::endl;
    loop_rate.sleep();
  }
}

MoveitBridge::MoveitBridge(){
  node = rclcpp::Node::make_shared("ros1_moveit_get_trajectory");
  isGetResult = false;
  jointStatesChatter = node->create_publisher<sensor_msgs::msg::JointState>("tm_moveit_joint_states", rclcpp::QoS(10));
  jointTargetChatter = node->create_publisher<sensor_msgs::msg::JointState>("tm_moveit_joint_target", rclcpp::QoS(10));
  cartesianChatter = node->create_publisher<geometry_msgs::msg::Pose>("tm_moveit_cartiesan_target", rclcpp::QoS(10));
  std::thread(&MoveitBridge::joint_trajectory_listen_thread, this,node).detach();
  std::thread(&MoveitBridge::current_position_listen_thread, this,node).detach();

}
void MoveitBridge::set_current_joint(std::vector<double> jointPosition){
  sensor_msgs::msg::JointState msg;
  //msg.header.stamp =  ros::Time::now();
  //msg.name = jointNames; 
  msg.position = jointPosition;
  MoveitBridge::wait_connect_success(jointStatesChatter);
  jointStatesChatter->publish(msg);
}
trajectory_msgs::msg::JointTrajectory MoveitBridge::get_trajectories(std::vector<double> jointTarget){
  sensor_msgs::msg::JointState msg;
 
  //msg.header.stamp =  ros::Time::now();
  
  //msg.name = jointNames; 
  msg.position = jointTarget;

  MoveitBridge::wait_connect_success(jointTargetChatter);
  
  jointTargetChatter->publish(msg);
  
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(200));
  
  while(!isGetResult){
    loop_rate.sleep();
    std::cout<<"wait result"<<std::endl;
  }
  isGetResult = false;
  return trajectory;
}
trajectory_msgs::msg::JointTrajectory MoveitBridge::get_trajectories(geometry_msgs::msg::Pose eeTarget){
  geometry_msgs::msg::Pose msg;
  msg = eeTarget;
  MoveitBridge::wait_connect_success(cartesianChatter);
  cartesianChatter->publish(msg);

  rclcpp::WallRate loop_rate(std::chrono::milliseconds(200));
   while(isGetResult){
    loop_rate.sleep();
  }
  isGetResult = false;
  return trajectory;
}
geometry_msgs::msg::Pose MoveitBridge::get_current_position(){
  return current_position;
}