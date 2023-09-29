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

#include "catch_ros2/catch.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

bool killed = false;

void signal_handler(int)
{
  killed = true;
}

// https://stackoverflow.com/questions/5919622/how-to-store-the-system-command-output-in-a-variable
class CheckAliveNodes
{
public:
  CheckAliveNodes():
    // run a command to get the list of nodes still alive
    fpipe_{static_cast<FILE*>(popen("ros2 daemon stop && ros2 node list", "r"))}
  {
    // Wait a little for command to complete to reduce broken pipe errors
    // Can't seem to fully eliminate them
    sleep(1.0);

    if (fpipe_)
    {
      char c = 0;

      // build result
      result_ = "";


      while (fread(&c, sizeof(c), 1, fpipe_))
      {
        result_ += c;
      }
    }
  }

  ~CheckAliveNodes()
  {
    // Close file pipe
    if (fpipe_) {pclose(fpipe_);}
  }

  // Return result of command
  std::string result() const {return result_;}

private:
  FILE* fpipe_ = NULL;

  std::string result_ = "FAILURE";
};

TEST_CASE("launch_remote_ssh_test", "[launch_remote_ssh]") {
  auto node = rclcpp::Node::make_shared(
    "tester_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // auto TEST_DURATION = 60.0;

  // if (node->has_parameter("test_duration"))
  // {
  //   TEST_DURATION = node->get_parameter("test_duration").as_double();
  // }
  // else
  // {
  //   TEST_DURATION = node->declare_parameter<double>("test_duration", 60.0);
  // }

  // size_t NUM_PARAMS = 0;

  // // Determine the number of parameters passed in for testing
  // while (true)
  // {
  //   if (node->has_parameter("param" + std::to_string(NUM_PARAMS + 1)))
  //   {
  //     NUM_PARAMS++;
  //   }
  //   else
  //   {
  //     break;
  //   }
  // }

  // RCLCPP_INFO_STREAM(node->get_logger(), "Testing " << NUM_PARAMS << " parameters");
  // REQUIRE(NUM_PARAMS != 0);

  // std::vector<std::shared_ptr<rclcpp::SyncParametersClient>> param_clients;

  // // Construct list of parameter clients
  // for (size_t i = 0; i < NUM_PARAMS; i++)
  // {
  //   param_clients.push_back(
  //     std::make_shared<rclcpp::SyncParametersClient>(
  //       node,
  //       "param" + std::to_string(i + 1) + "_node"
  //     )
  //   );
  // }

  rclcpp::Time start_time = rclcpp::Clock().now();

  // size_t param_ndx = 0;

  // // Keep test running only for the length test duration (in seconds)
  // while (
  //   rclcpp::ok() &&
  //   ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  // )
  // {
  //   if (param_clients.at(param_ndx)->wait_for_service(0s))
  //   {
  //     // Get parameter from remote parameter node
  //     const auto param_received = param_clients.at(param_ndx)->get_parameters({"param0"})[0];

  //     // Get same parameter on this node
  //     const auto param_input_name = "param" + std::to_string(param_ndx + 1);
  //     const auto param_input = node->get_parameter(param_input_name);

  //     RCLCPP_INFO_STREAM(node->get_logger(), param_input_name << " found:");
  //     RCLCPP_INFO_STREAM(
  //       node->get_logger(),
  //       "Input:\tType(" << param_input.get_type_name()
  //       << ")\tValue(" << param_input.value_to_string() << ")"
  //     );
  //     RCLCPP_INFO_STREAM(
  //       node->get_logger(),
  //       "Received:\tType(" << param_received.get_type_name()
  //       << ")\tValue(" << param_received.value_to_string() << ")"
  //     );

  //     // Test that the types and values match
  //     CHECK(param_received.get_type() == param_input.get_type());
  //     CHECK(param_received.get_parameter_value() == param_input.get_parameter_value());

  //     param_ndx++;

  //     if (param_ndx >= NUM_PARAMS)
  //     {
  //       break;
  //     }
  //   }

  //   rclcpp::spin_some(node);
  // }

  // // Ensure all parameters were checked
  // CHECK(NUM_PARAMS == param_ndx);

  RCLCPP_INFO_STREAM(node->get_logger(), "--------------------ACTION REQUIRED--------------------");
  RCLCPP_INFO_STREAM(node->get_logger(), "Please terminate the launch file to continue the test.");
  RCLCPP_INFO_STREAM(node->get_logger(), "--------------------ACTION REQUIRED--------------------");


  // Ignore SIGINTs and SIGTERMs while checking for nodes to be killed
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  
  struct sigaction sigTermHandler;
  sigTermHandler.sa_handler = signal_handler;
  sigemptyset(&sigTermHandler.sa_mask);
  sigTermHandler.sa_flags = 0;
  sigaction(SIGTERM, &sigTermHandler, NULL);

  // Wait until a SIGINT is received
  while (!killed) {}

  // Check that the remote nodes are properly shut down
  start_time = rclcpp::Clock().now();
  bool nodes_killed = false;
  int count = 0;

  // ROS will send a SIGKILL 10 seconds after a SIGTERM (5 seconds after SIGINT)
  // So we only have about 15 seconds to observe the nodes being killed
  // rclcpp::Node::get_node_names() is not fast enough to detect this
  // Instead, running 'ros2 daemon stop && ros2 node list' via the another process
  // detects the shut down nodes much faster.
  while (
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(8.0))
  )
  {

    rclcpp::spin_some(node);

    // Get the output from the command line
    const CheckAliveNodes alive_nodes {};

    // Make sure we didn't fail to get the output
    REQUIRE(alive_nodes.result().find("FAILURE") == std::string::npos);

    // check output for the remote node names
    if (
      (alive_nodes.result().find("/param_nodes") == std::string::npos)
      &&
      (alive_nodes.result().find("/server_nodes") == std::string::npos)
    )
    {
      // Remote node names are not found, increment count
      count++;
    }
    else
    {
      // Found remote node names, reset count
      count = 0;
    }

    // Require nodes to not be seen multiple times before declaring them "killed"
    if (count >= 5)
    {
      nodes_killed = true;
      break;
    }
  }

  REQUIRE(nodes_killed);
}