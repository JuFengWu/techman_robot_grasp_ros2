#include "../include/send_command.hpp"
#include<cmath>
namespace tm_driver{

  SendCommand::SendCommand(rclcpp::Node::SharedPtr node){
    this->node = node;
    this->jointCommabdPublish = node->create_publisher<tm_msgs::msg::JointTrajectorys>("joint_trajectory_msgs",rclcpp::QoS(100));
    auto listenNode = rclcpp::Node::make_shared("MyMsgSuber");
    auto callBack =[this](const tm_msgs::msg::RobotStatus::SharedPtr robotStatus) -> void
        {
          this->isProcessCmd = robotStatus->is_process_cmd;
          this->currentCommanderId = robotStatus->commander_id;
        };
    auto subscription = listenNode->create_subscription<tm_msgs::msg::RobotStatus>("tm_motor_states", 100, callBack);

    //open thread and spin listenNode
  }

  void SendCommand::send_joint_position(std::vector<std::vector<double>> jointPosition){

    auto motorStatusMsg = std::make_shared<tm_msgs::msg::JointTrajectorys>();
    motorStatusMsg->robot_id = 0;
    motorStatusMsg->joint_control_mode = 0;
    motorStatusMsg->joint_trajectory.points.resize(jointNumber);
    

    for(int i=0;i<jointNumber;i++){
      motorStatusMsg->joint_trajectory.points[i].positions.clear();
      for(unsigned int j=0;j<jointPosition[i].size();j++){
        motorStatusMsg->joint_trajectory.points[i].positions.push_back(jointPosition[i][j]);
      }
    }
    std::cout<<"ready to send goal!!"<<std::endl;
    rclcpp::WallRate loop_rate(1);
    
    while(true) {
      if(!isProcessCmd){
        motorStatusMsg->robot_id ++;
        RCLCPP_INFO(node->get_logger(),"Pub something");
        jointCommabdPublish->publish(*motorStatusMsg);
      }
      else if(currentCommanderId == myCommanderId){
        break;
      }
      else{
        std::cout<<"other is controlling robot, wait a momnet"<<std::endl;
      }
      loop_rate.sleep();
    }
    
  }
  void SendCommand::send_joint_velocity(std::vector<std::vector<double>> jointVelocity){

    auto motorStatusMsg = std::make_shared<tm_msgs::msg::JointTrajectorys>();
    motorStatusMsg->robot_id =0;
    motorStatusMsg->joint_control_mode = 0;
    motorStatusMsg->joint_trajectory.points.resize(jointNumber);
    

    for(int i=0;i<jointNumber;i++){
      motorStatusMsg->joint_trajectory.points[i].velocities.clear();
      for(unsigned int j=0;j<jointVelocity[i].size();j++){
        motorStatusMsg->joint_trajectory.points[i].velocities.push_back(jointVelocity[i][j]);
      }
    }
    std::cout<<"ready to send goal!!"<<std::endl;
    jointCommabdPublish->publish(*motorStatusMsg);

  }

}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  
  std::vector<std::vector<double>> jointPoints;
  double counter = 0;
  for(int j=0;j<6;j++){
    std::vector<double> jointPosition;
    for(int i=0;i<10000;i++){
      jointPosition.push_back(sin(counter));
      counter+=0.0005;
      
    }
    counter = 0;
    jointPoints.push_back(jointPosition);
  }
  
  std::cout<<"jointPoints[i].size() is "<<jointPoints[0].size()<<std::endl;

    for(unsigned int j=0;j<jointPoints[0].size();j++){
        std::cout<<"jointPoints[i][j]"<<jointPoints[0][j]<<std::endl;
      }


  auto node = rclcpp::Node::make_shared("MyMsgSuber");
  std::unique_ptr<tm_driver::SendCommand> sendCommand = std::make_unique<tm_driver::SendCommand>(node);

  
  sendCommand->send_joint_position(jointPoints);
  //std::cout<<"AAAAAAA"<<std::endl;
  //rclcpp::WallRate loop_rate(1);
  //while(true)
 // {
  //  std::cout<<"in loop rate"<<std::endl;
   // loop_rate.sleep();
  //}
  
  
  std::cout<<"finish!"<<std::endl;
  

}
