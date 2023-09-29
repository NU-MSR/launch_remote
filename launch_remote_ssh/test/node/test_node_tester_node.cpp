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
using Catch::Matchers::Equals;

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

template<typename T>
void check_array(std::vector<T> actual, std::vector<T> expected)
{
  REQUIRE(actual.size() == expected.size());
  for (size_t i = 0; i < expected.size(); i++)
  {
    REQUIRE(actual.at(i) == expected.at(i));
  }
}

void check_string_array(std::vector<std::string> actual, std::vector<std::string> expected)
{
  REQUIRE(actual.size() == expected.size());
  for (size_t i = 0; i < expected.size(); i++)
  {
    REQUIRE_THAT(actual.at(i), Equals(expected.at(i)));
  }
}

TEST_CASE("node_remote_ssh_test", "[node_remote_ssh]") {
  auto node = rclcpp::Node::make_shared(
    "tester_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto TEST_DURATION = 20.0;

  if (node->has_parameter("test_duration"))
  {
    TEST_DURATION = node->get_parameter("test_duration").as_double();
  }
  else
  {
    TEST_DURATION = node->declare_parameter<double>("test_duration", TEST_DURATION);
  }

  REQUIRE(node->has_parameter("num_params1"));
  const size_t NUM_PARAMS1 = node->get_parameter("num_params1").as_int();

  auto params1_client =
    std::make_shared<rclcpp::SyncParametersClient>(node, "/param_nodes/param_node1");

  
  rclcpp::Time start_time = rclcpp::Clock().now();

  bool service_found = false;

  // Keep test running only for the length test duration (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {

    if (params1_client->wait_for_service(0s)) {
      service_found = true;
      break;
    }

    rclcpp::spin_some(node);
  }

  REQUIRE(service_found);

  std::vector<std::string> params1_names (NUM_PARAMS1);

  for (size_t i = 0; i < NUM_PARAMS1; i++)
  {
    params1_names.at(i) = "param" + std::to_string(i);
  }

  const auto params1_received = params1_client->get_parameters(params1_names, 10s);

  REQUIRE(params1_received.size() == NUM_PARAMS1);

  REQUIRE(params1_received.at(0).get_type() == rclcpp::ParameterType::PARAMETER_BOOL);
  REQUIRE(params1_received.at(0).as_bool() == false);

  REQUIRE(params1_received.at(1).get_type() == rclcpp::ParameterType::PARAMETER_STRING);
  REQUIRE_THAT(params1_received.at(1).as_string(), Equals("happy!"));

  REQUIRE(params1_received.at(2).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER);
  REQUIRE(params1_received.at(2).as_int() == -256);

  REQUIRE(params1_received.at(3).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE);
  REQUIRE(params1_received.at(3).as_double() == 42.42);

  REQUIRE(params1_received.at(4).get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
  REQUIRE(params1_received.at(4).as_bool_array().size() == 5);
  check_array(params1_received.at(4).as_bool_array(), {false, true, false, true , true});

  REQUIRE(params1_received.at(5).get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  REQUIRE(params1_received.at(5).as_string_array().size() == 5);
  check_string_array(params1_received.at(5).as_string_array(), {"I", "like", "when", "things", "work"});

  REQUIRE(params1_received.at(6).get_type() == rclcpp::ParameterType::PARAMETER_STRING);
  REQUIRE_THAT(params1_received.at(6).as_string(), Equals("But this is a single string"));

  REQUIRE(params1_received.at(7).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  REQUIRE(params1_received.at(7).as_integer_array().size() == 9);
  check_array(params1_received.at(7).as_integer_array(), {9,8,7,6,5,4,3,2,1});

  REQUIRE(params1_received.at(8).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  REQUIRE(params1_received.at(8).as_double_array().size() == 9);
  check_array(params1_received.at(8).as_double_array(), {9.0,8.0,7.0,6.0,5.0,4.0,3.0,2.0,1.0});

  REQUIRE(params1_received.at(9).get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  REQUIRE(params1_received.at(9).as_string_array().size() == 6);
  check_string_array(params1_received.at(9).as_string_array(), {"This", "is", "not", "a", "single", "string"});

  REQUIRE(params1_received.at(10).get_type() == rclcpp::ParameterType::PARAMETER_STRING);
  REQUIRE_THAT(params1_received.at(10).as_string(), Equals("hello world"));

  REQUIRE(params1_received.at(11).get_type() == rclcpp::ParameterType::PARAMETER_BOOL);
  REQUIRE(params1_received.at(11).as_bool() == true);

  REQUIRE(params1_received.at(12).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER);
  REQUIRE(params1_received.at(12).as_int() == 45);

  REQUIRE(params1_received.at(13).get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
  REQUIRE(params1_received.at(13).as_bool_array().size() == 5);
  check_array(params1_received.at(13).as_bool_array(), {false, true, false, true , false});

  REQUIRE(params1_received.at(14).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  REQUIRE(params1_received.at(14).as_integer_array().size() == 9);
  check_array(params1_received.at(14).as_integer_array(), {1,2,3,4,5,6,7,8,9});

  REQUIRE(params1_received.at(15).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  REQUIRE(params1_received.at(15).as_double_array().size() == 9);
  check_array(params1_received.at(15).as_double_array(), {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0});

  REQUIRE(params1_received.at(16).get_type() == rclcpp::ParameterType::PARAMETER_BOOL);
  REQUIRE(params1_received.at(16).as_bool() == true);

  REQUIRE(params1_received.at(17).get_type() == rclcpp::ParameterType::PARAMETER_STRING);
  REQUIRE_THAT(params1_received.at(17).as_string(), Equals("movin and groovin"));

  REQUIRE(params1_received.at(18).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER);
  REQUIRE(params1_received.at(18).as_int() == 89745);

  REQUIRE(params1_received.at(19).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE);
  REQUIRE(params1_received.at(19).as_double() == -89.45);

  REQUIRE(params1_received.at(20).get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
  REQUIRE(params1_received.at(20).as_bool_array().size() == 6);
  check_array(params1_received.at(20).as_bool_array(), {true, true, true, true, true, false});

  REQUIRE(params1_received.at(21).get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  REQUIRE(params1_received.at(21).as_string_array().size() == 4);
  check_string_array(params1_received.at(21).as_string_array(), {"it", "is", "party", "time"});
  
  REQUIRE(params1_received.at(22).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  REQUIRE(params1_received.at(22).as_integer_array().size() == 3);
  check_array(params1_received.at(22).as_integer_array(), {123, 456, 789});

  REQUIRE(params1_received.at(23).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  REQUIRE(params1_received.at(23).as_double_array().size() == 3);
  check_array(params1_received.at(23).as_double_array(), {9.87, 6.54, 3.21});

  

  // if (param_clients.at(param_ndx)->wait_for_service(0s))
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