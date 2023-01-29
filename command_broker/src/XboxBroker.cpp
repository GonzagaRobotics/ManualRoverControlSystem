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
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

class XboxBroker : public rclcpp::Node
{
public:
  XboxBroker() : Node("XboxBroker")
  {
    // Quality of Service (QoS) profile for the subscription
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    // Subscribing to the "joy" topic
    auto callback =
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      {
        // Modify the data here
        std_msgs::msg::Float32MultiArray modified_data;

    float left_trigger = msg->axes[2];
    float right_trigger = msg->axes[5];
    float left_shoulder = msg->buttons[4];
    float right_shoulder = msg->buttons[5];
    float dpad_lr = msg->axes[6];

    modified_data.data = {left_trigger, right_trigger, left_shoulder, right_shoulder, dpad_lr};
      // modified_data.data = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5], msg->axes[6], msg->axes[7]};

        // Publish the modified data to the "motor_command" topic
        pub_->publish(modified_data);
      };

    sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",qos, callback);

    // Quality of Service (QoS) profile for the publisher
    auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    // Creating the publisher for the "motor_command" topic
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_command", qos_pub);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxBroker>());
  rclcpp::shutdown();
  return 0;
}

