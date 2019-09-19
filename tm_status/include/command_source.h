#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
class CommandSourceInterface{
  public:
    virtual int get_loop_time()=0;
    virtual void get_outside_command()=0;
    virtual void run_command(sensor_msgs::msg::JointState& msg,rclcpp::Clock::SharedPtr clock)=0;
};

class TestMoveCommand: public CommandSourceInterface{
  private:
    double counter = 0.0;
  public:
    int get_loop_time()override;
    void get_outside_command() override;
    void run_command(sensor_msgs::msg::JointState& msg,rclcpp::Clock::SharedPtr clock) override;
};
