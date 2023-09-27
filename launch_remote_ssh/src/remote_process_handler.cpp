// BSD 3-Clause License
//
// Copyright (c) 2023, Northwestern University MSR
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Nick Morales

#include <string>
#include "rclcpp/rclcpp.hpp"

// RAII node to terminate the indicated process
class RemoteProcessHandler : public rclcpp::Node
{
public:
  RemoteProcessHandler()
  : Node("remote_process_handler"),
  // get screen PID to kill
  screen_pid_{declare_parameter<std::string>("screen_pid")},
  // construct kill command
  kill_command_{"screen -S " + screen_pid_ + " -X quit"}
  {
  }
  ~RemoteProcessHandler()
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

  auto node = std::make_shared<RemoteProcessHandler>();

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