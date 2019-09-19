#include "sensor_msgs/msg/joint_state.hpp"
class ModelInterface{

  public:
    virtual void set_model_name(sensor_msgs::msg::JointState& msg) =0;
    void set_model_initial_pose(sensor_msgs::msg::JointState& msg,double allInitialPose);
};


class TmGraspModel :public ModelInterface{
  public:
    virtual void set_model_name(sensor_msgs::msg::JointState& msg) override;
};