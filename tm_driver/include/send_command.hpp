
namespace tm_driver{
  
  class SendCommand{
  public:
    SendCommand();
    void send_joint_position();
    void send_joint_velocity();
    double check_process();
    int get_error_code();
  };
}

