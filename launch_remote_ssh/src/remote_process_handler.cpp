#include <string>
#include "rclcpp/rclcpp.hpp"

// RAII node to terminate the indicated process
class RemoteLaunchHandler : public rclcpp::Node
{
public:
  RemoteLaunchHandler()
  : Node("remote_launch_handler"),
  // get screen PID to kill
  screen_pid_{declare_parameter<std::string>("screen_pid")},
  // construct kill command
  kill_command_{"screen -S " + screen_pid_ + " -X quit"}
  {
  }
  ~RemoteLaunchHandler()
  {
    // execute kill command
    system(kill_command_.c_str());
  }

  const std::string screen_pid() const
  {
    return screen_pid_;
  }
  
private:
  const std::string screen_pid_ = "";
  const std::string kill_command_ = "";
};

int main(int argc, char * argv[])
{

  // Init ROS
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RemoteLaunchHandler>();

  try
  {
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "\nTERMINATING: " << node->screen_pid());

    // Shutdown ROS
    rclcpp::shutdown();
  }
  catch (const rclcpp::exceptions::RCLError & e)
  {
  }

  return 0;
}