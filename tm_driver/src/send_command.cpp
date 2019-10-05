#include "../include/send_command.hpp"
#include<cmath>
namespace tm_driver{

  SendCommand::SendCommand(rclcpp::Node::SharedPtr node){
    this->node = node;
    actionClient = rclcpp_action::create_client<JointTrajectory>(node, "send_command");
    if (!actionClient->wait_for_action_server(std::chrono::seconds(20))) {
      RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    }
    goalMsg = JointTrajectory::Goal();
  }

  void feedback_callback(SendCommand *sendCommand, rclcpp_action::ClientGoalHandle<JointTrajectory>::SharedPtr,
                    const std::shared_ptr<const JointTrajectory::Feedback> feedback){
    sendCommand->processPersent = feedback->process_persent;
    std::cout<<"process persent is "<<sendCommand->processPersent<<std::endl;
  }
  int SendCommand::check_result_correct(rclcpp_action::ClientGoalHandle<JointTrajectory>::WrappedResult wrappedResult){
    switch (wrappedResult.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        std::cout<<"Goal was aborted"<<std::endl;
        return 1;
      case rclcpp_action::ResultCode::CANCELED:
        std::cout<<"Goal was canceled"<<std::endl;
        return 1;
      default:
        std::cout<< "Unknown result code"<<std::endl;
        return 1;
    }
    return 0;
  }
  int SendCommand::send_goal(){
    is_success = false;
    auto sendGoalOptions = rclcpp_action::Client<JointTrajectory>::SendGoalOptions();
    sendGoalOptions.feedback_callback = std::bind(feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    //sendGoalOptions.feedback_callback = feedback_callback;
    auto goalHandleFuture = actionClient->async_send_goal(goalMsg, sendGoalOptions);

    if (rclcpp::spin_until_future_complete(node, goalHandleFuture) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      std::cout<< "send goal call failed "<<std::endl;
      return 1;
    }
    rclcpp_action::ClientGoalHandle<JointTrajectory>::SharedPtr goalHandle = goalHandleFuture.get();
    if (!goalHandle) {
      std::cout<< "Goal was rejected by server "<<std::endl;
      return 1;
    }
    // Wait for the server to be done with the goal
    auto resultFuture = actionClient->async_get_result(goalHandle);
    std::cout<< "Waiting for result"<<std::endl;
    if (rclcpp::spin_until_future_complete(node, resultFuture) !=rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      std::cout<<"get result call failed !"<<std::endl;
      return 1;
    }

    rclcpp_action::ClientGoalHandle<JointTrajectory>::WrappedResult wrappedResult = resultFuture.get();
    if(1 == check_result_correct(wrappedResult)){
        return 1;
    }
    
    std::cout<<  "result received"<<std::endl;
    this->error_code = wrappedResult.result-> error_code;
    this->is_success = wrappedResult.result-> is_success;
    
    actionClient.reset();
    node.reset();
    return 0;
  }
  int SendCommand::send_joint_position(std::vector<std::vector<double>> jointPosition){
    goalMsg.robot_id = 0;
    goalMsg.joint_control_mode = 0;
    for(int i=0;i<6;i++){
      for(unsigned int j=0;j<jointPosition[i].size();j++){
        goalMsg.joint_trajectory.points[i].positions[j] = jointPosition[i][j];
      }
    }
    std::cout<<"ready to send goal!!"<<std::endl;
    return send_goal();
  }
  int SendCommand::send_joint_velocity(std::vector<std::vector<double>> jointVelocity){
    goalMsg.robot_id = 0;
    goalMsg.joint_control_mode = 0;
    for(int i=0;i<6;i++){
      for(unsigned int j=0;j<jointVelocity[i].size();j++){
        goalMsg.joint_trajectory.points[i].velocities[j] = jointVelocity[i][j];
      }
    }
    std::cout<<"ready to send goal!!"<<std::endl;
    return send_goal();
  }
  double SendCommand::check_process(){
    return this->processPersent;
  }
  int SendCommand::get_error_code(){
    return this->error_code;
  }
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  
  std::vector<std::vector<double>> jointPoints;
  for(int j=0;j<6;j++){
    std::vector<double> jointPosition;
    for(int i=0;i<10000;i++){
      jointPosition.push_back(sin(i/10000));
    }
    jointPoints.push_back(jointPosition);
  }
  auto node = rclcpp::Node::make_shared("MyMsgSuber");
  std::unique_ptr<tm_driver::SendCommand> sendCommand = std::make_unique<tm_driver::SendCommand>(node);
  int returnValue = sendCommand->send_joint_position(jointPoints);
  std::cout<<"return Vvlue is "<<returnValue<<std::endl;
  std::cout<<"finish!"<<std::endl;

}
