class RobotSimulatorInterface{
  public:
    /// Pointer to model.
    gazebo::physics::ModelPtr model;//TODO: change to set, get
  protected:
    virtual void set_model_joint();
    virtual void create_topic();
    virtual void create_command_action();
    virtual void set_command_to_gazebo();
    virtual void get_current_state_to_publish();
    std::vector<gazebo::physics::JointPtr> gazeboJoint;
    float initial_joints_value;
    gazebo_ros::Node::SharedPtr joint_status_publish_node;
    std::shared_ptr<rclcpp::Publisher<tm_msgs::msg::RobotStatus>> motor_status_pub;
}

class TmRobotSimulator : public RobotSimulatorInterface{
  private:
    int joint_number=6;
    void message_publish()
  public:
    void set_model_joint() override;
    void create_topic() override;
    void create_command_action() override;
    void set_command_to_gazebo() override;
    void get_current_state_to_publish() override;
  
}
