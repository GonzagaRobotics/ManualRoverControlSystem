// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.




#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

class CommandExposer : public rclcpp::Node
{
public:
  CommandExposer() : Node("CommandExposer")
  {
    // Quality of Service (QoS) profile for the subscription
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    // Subscribing to the "motor_command" topic
    auto callback =
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) -> void
      {
        // Don't modify the data here
        std_msgs::msg::Float32MultiArray unmodified_data = msg.data;
 
        // Publish the unmodified data to the "motor_command_exposed" topic
        pub_->publish(unmodified_data);
      };

    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("motor_command",qos, callback);

    // Quality of Service (QoS) profile for the publisher
    auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    // Creating the publisher for the "motor_command_exposed" topic
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_command_exposed", qos_pub);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray2>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandExposer>());
  rclcpp::shutdown();
  return 0;
}

