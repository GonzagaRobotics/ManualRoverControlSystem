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









































// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joy.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>

// class XboxBroker : public rclcpp::Node
// {
// public:
//   XboxBroker() : Node("XboxBroker")
//   {
//     // Create a subscription to the "joy" topic
//     auto joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10), std::bind(&XboxBroker::joy_callback, this, std::placeholders::_1));



//     // Create a publisher for the "motor_command" topic
//     motor_command_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_command", 10);
//   }

// private:
//   void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
//   {
//     // Process the joy message here

//     // Create a message to publish
//     auto motor_command_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();

//     // Fill in the message data
//     float left_trigger = msg->axes[3];
//     float right_trigger = msg->axes[6];
//     float left_shoulder = msg->buttons[5];
//     float right_shoulder = msg->buttons[6];
//     float dpad_lr = msg->axes[7];

//     motor_command_msg->data = {left_trigger, right_trigger, left_shoulder, right_shoulder, dpad_lr};

//     // Publish the message
//     motor_command_pub_->publish(std::move(motor_command_msg));
//   }

//   rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_command_pub_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto xbox_broker = std::make_shared<XboxBroker>();
//   rclcpp::spin(xbox_broker);
//   rclcpp::shutdown();
//   return 0;
// }






























// #include <functional>
// #include <memory>
// #include <chrono>
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include <sensor_msgs/msg/joy.hpp>

// using namespace std::chrono_literals;

// using std::placeholders::_1;


// class MinimalSubscriber : public rclcpp::Node
// {
// public:
//   MinimalSubscriber()
//   : Node("XboxBroker")
//   {
//     subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
//     std::bind(&MinimalSubscriber::topic_callback, this, _1));

    
    

//   //  rmw_publisher_t publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_command", 10);
//   //   timer_ = this->create_wall_timer(
//   //     500ms, std::bind(motor_command_values, this));
//   }
  
// private:
//   void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) const
//   {
//     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
//   }

//   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

// };


// class MinimalPublisher : public rclcpp::Node
// {
// public:
//   MinimalPublisher()
//   : Node("XboxBroker"), count_(0)
//   {
//     publisher_ = this->create_publisher<std::vector<float>>("motor_command", 10);
//     timer_ = this->create_wall_timer(
//       500ms, std::bind(&MinimalPublisher::timer_callback, this));
//   }

// private:
//   void timer_callback()
//   {
//      std::vector<float> message = {5.2, 2, 7.3, 4, 0};
//     // message.data = "Hello, world! " + std::to_string(count_++);
//     // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_->publish(message);
//   }
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<std::vector<float>>::SharedPtr publisher_;
//   size_t count_;
// };


// int main(int argc, char * argv[])
// {
  
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalSubscriber>());
  

//   sensor_msgs::msg::Joy::SharedPtr joy_msg;
//     //std::map<std::string, int64_t> axis_linear_map;
//     float left_trigger = joy_msg->axes[3];
//     float right_trigger = joy_msg->axes[6];
//     float left_shoulder = joy_msg->buttons[5];
//     float right_shoulder = joy_msg->buttons[6];
//     float dpad_lr = joy_msg->axes[7];

//    // float motor_command_values[] = {left_trigger, right_trigger, left_shoulder, right_shoulder, dpad_lr};
   
//    rclcpp::spin(std::make_shared<MinimalPublisher>());


//   rclcpp::shutdown();
//   return 0;
// }
