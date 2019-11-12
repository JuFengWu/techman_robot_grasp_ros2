#include "../include/moveit_bridge.h"

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  std::unique_ptr<MoveitBridge> moveitBrdige = std::make_unique<MoveitBridge>();

  std::vector<double> initialJointPosition{0.0,0.0,90.0,0.0,90.0,0.0};
  moveitBrdige->set_current_joint(initialJointPosition);
  
  std::vector<double> targetJointPosition1{21.0,32.0,90.0,43.0,90.0,8.0};
  auto trajectories = moveitBrdige->get_trajectories(targetJointPosition1);

  std::cout<<"trajectories are :"<<std::endl;
  for(unsigned int i=0; i<trajectories.points[0].positions.size();i++){
    for(unsigned int j=0;i<trajectories.points.size();j++){
        std::cout<<"J"<<j<<":"<<trajectories.points[j].positions[i]<<",";
    }
    std::cout<<std::endl;
  }

  auto currentPosition = moveitBrdige->get_current_position();
  
  geometry_msgs::msg::Pose cmdPosition = currentPosition;
  cmdPosition.position.x +=0.5;
  cmdPosition.position.y +=0.5;
  cmdPosition.position.z +=0.5;

  trajectories = moveitBrdige->get_trajectories(cmdPosition);
  std::cout<<"trajectories are :"<<std::endl;
  for(unsigned int i=0; i<trajectories.points[0].positions.size();i++){
    for(unsigned int j=0;i<trajectories.points.size();j++){
        std::cout<<"J"<<j<<":"<<trajectories.points[j].positions[i]<<",";
    }
    std::cout<<std::endl;
  }
}