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
#include <std_msgs/msg/float32.hpp>

class CommandExposer : public rclcpp::Node
{
public:
  CommandExposer() : Node("CommandExposer")
  {
    // Quality of Service (QoS) profile for the subscription
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    // Subscribing to the "motor_command" topic(s)
    auto callback_lt =
      [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
      {
        pub_lt_->publish(*msg);
      };
    auto callback_rt =
      [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
      {
        pub_rt_->publish(*msg);
      };
    auto callback_lb =
      [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
      {
        pub_lb_->publish(*msg);
      };
    auto callback_rb =
      [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
      {
        pub_rb_->publish(*msg);
      };
    auto callback_dpad_lr =
      [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
      {
        pub_dpad_lr_->publish(*msg);
      };
    auto callback_pause =
      [this](const std_msgs::msg::Float32::SharedPtr msg) -> void
      {
        pub_pause_->publish(*msg);
      };

    sub_lt = this->create_subscription<std_msgs::msg::Float32>("motor_command/left_trigger", qos, callback_lt);
    sub_rt = this->create_subscription<std_msgs::msg::Float32>("motor_command/right_trigger", qos, callback_rt);
    sub_lb = this->create_subscription<std_msgs::msg::Float32>("motor_command/left_shoulder", qos, callback_lb);
    sub_rb = this->create_subscription<std_msgs::msg::Float32>("motor_command/right_shoulder", qos, callback_rb);
    sub_dpad_lr = this->create_subscription<std_msgs::msg::Float32>("motor_command/dpad_lr", qos, callback_dpad_lr);
    sub_pause = this->create_subscription<std_msgs::msg::Float32>("motor_command/dpad_lr", qos, callback_pause);

    // Quality of Service (QoS) profile for the publisher
    auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    // Creating the publisher for the "motor_command_exposed" topic
    pub_lt_ = this->create_publisher<std_msgs::msg::Float32>("motor_command_exposed/left_trigger", qos_pub);
    pub_rt_ = this->create_publisher<std_msgs::msg::Float32>("motor_command_exposed/right_trigger", qos_pub);
    pub_lb_ = this->create_publisher<std_msgs::msg::Float32>("motor_command_exposed/left_shoulder", qos_pub);
    pub_rb_ = this->create_publisher<std_msgs::msg::Float32>("motor_command_exposed/right_shoulder", qos_pub);
    pub_dpad_lr_ = this->create_publisher<std_msgs::msg::Float32>("motor_command_exposed/dpad_lr", qos_pub);
    pub_pause_ = this->create_publisher<std_msgs::msg::Float32>("motor_command_exposed/pause", qos_pub);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lt;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rt;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lb;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rb;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_dpad_lr;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_pause;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_lt_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rt_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_lb_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rb_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dpad_lr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pause_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandExposer>());
  rclcpp::shutdown();
  return 0;
}

