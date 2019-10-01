class RobotSimulatorInterface{
  public:
    /// Pointer to model.
    gazebo::physics::ModelPtr model;//TODO: change to set, get
    const int jointPositionControl =0;
    const int jointVelocityControl =1;
    int controlMode = 0;
  protected:
    virtual void set_model_joint();
    virtual void create_topic();
    virtual void create_command_action();
    virtual void set_command_to_gazebo();
    std::vector<gazebo::physics::JointPtr> gazeboJoint;
    float initial_joints_value;
    gazebo_ros::Node::SharedPtr rosNode;
    std::shared_ptr<rclcpp::Publisher<tm_msgs::msg::RobotStatus>> motorStatusPublish;
}

class TmRobotSimulator : public RobotSimulatorInterface{
  private:
    int jointNumber=6;
    void message_publish();
    std::vector<std::vector<double>> tajectoryVelocity;
    std::vector<std::vector<double>> tajectoryPosition;
  public:
    void set_model_joint() override;
    void create_topic() override;
    void create_command_action() override;
    void set_command_to_gazebo() override;
  
}
