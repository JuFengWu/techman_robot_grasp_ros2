#include"../include/listen_robot_status.hpp"

using namespace std::chrono_literals;

namespace tm_driver{
  void ListenRobotSatus::listen_thread(rclcpp::Node::SharedPtr node){
    
    auto callBack =[this](const tm_msgs::msg::RobotStatus::SharedPtr robotStatus) -> void
        {
          std::cout<<"I hear something"<<std::endl;
          for(int i=0;i<jointNumber;i++){
            //std::cout<<"i is "<< i <<std::endl;
            //std::cout<<"robotStatus->current_joint_position is"<<robotStatus->current_joint_position[i]<<std::endl;
            jointPostion[i] = robotStatus->current_joint_position[i];
            //std::cout<<"jointPostion[i] is"<<jointPostion[i]<<std::endl;
            jointVelocity[i] = robotStatus->current_joint_velocity[i];
            jointTorque[i] = robotStatus->current_joint_force[i];
          }
        };
     auto subscription = node->create_subscription<tm_msgs::msg::RobotStatus>("tm_motor_states", 100, callBack);

    rclcpp::Rate sleepRate(loopRate);
    while (isListen){      
      rclcpp::spin_some(node);  
      sleepRate.sleep();
    }
    
    rclcpp::shutdown();
  }
  ListenRobotSatus::ListenRobotSatus(rclcpp::Node::SharedPtr node,float loopRate){
      
      isListen = true;
      this->loopRate = loopRate;
      jointPostion.resize(jointNumber);
      jointVelocity.resize(jointNumber);
      jointTorque.resize(jointNumber);
      std::thread(&ListenRobotSatus::listen_thread, this,node).detach();
    }
  
  ListenRobotSatus::~ListenRobotSatus(){
      isListen = false;
  }
  void ListenRobotSatus::set_loop_rate(float loopRate){
      this->loopRate = loopRate;
  }
  std::vector<double> ListenRobotSatus::get_joint_position(){
      return jointPostion;
  }
  std::vector<double> ListenRobotSatus::get_joint_velocity(){
      return jointVelocity;
  }
  std::vector<double> ListenRobotSatus::get_joint_torque(){
      return jointTorque;
  }
}
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("MyMsgSuber");
  std::unique_ptr<tm_driver::ListenRobotSatus> listenRobotSatus = std::make_unique<tm_driver::ListenRobotSatus>(node,0.5);
  rclcpp::Rate sleepRate(1s);
  while(true){
      auto position = listenRobotSatus->get_joint_position();
      auto vleocity = listenRobotSatus->get_joint_velocity();
      auto torque = listenRobotSatus->get_joint_torque();
      std::cout<<"--------------main loop------------"<<std::endl;
      std::cout<<"joint position is"<<position[0]<<","<<position[1]<<","<<position[2]<<","<<position[3]<<","<<position[4]<<","<<position[5]<<std::endl;
      std::cout<<"joint vleocity is"<<vleocity[0]<<","<<vleocity[1]<<","<<vleocity[2]<<","<<vleocity[3]<<","<<vleocity[4]<<","<<vleocity[5]<<std::endl;
      std::cout<<"joint torque is"<<torque[0]<<","<<torque[1]<<","<<torque[2]<<","<<torque[3]<<","<<torque[4]<<","<<torque[5]<<std::endl;
      sleepRate.sleep();
  }
}