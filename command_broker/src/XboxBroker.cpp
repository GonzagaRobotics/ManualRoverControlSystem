#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32.hpp>

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
        std_msgs::msg::Float32 left_trigger;
        left_trigger.data = msg->axes[2];

        std_msgs::msg::Float32 right_trigger;
        right_trigger.data = msg->axes[5];

        std_msgs::msg::Float32 left_shoulder;
        left_shoulder.data = msg->buttons[4];

        std_msgs::msg::Float32 right_shoulder;
        right_shoulder.data = msg->buttons[5];

        std_msgs::msg::Float32 dpad_lr;
        dpad_lr.data = msg->axes[6];

        // Publish the modified data to the "motor_command" topic
        pub_left_trigger_->publish(left_trigger);
        pub_right_trigger_->publish(right_trigger);
        pub_left_shoulder_->publish(left_shoulder);
        pub_right_shoulder_->publish(right_shoulder);
        pub_dpad_lr_->publish(dpad_lr);
      };

    sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",qos, callback);

    // Quality of Service (QoS) profile for the publisher
    auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    // Creating the publishers for the individual values
    pub_left_trigger_ = this->create_publisher<std_msgs::msg::Float32>("motor_command/left_trigger", qos_pub);
    pub_right_trigger_ = this->create_publisher<std_msgs::msg::Float32>("motor_command/right_trigger", qos_pub);
    pub_left_shoulder_ = this->create_publisher<std_msgs::msg::Float32>("motor_command/left_shoulder", qos_pub);
    pub_right_shoulder_ = this->create_publisher<std_msgs::msg::Float32>("motor_command/right_shoulder", qos_pub);
    pub_dpad_lr_ = this->create_publisher<std_msgs::msg::Float32>("motor_command/dpad_lr", qos_pub);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_trigger_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_trigger_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_shoulder_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_shoulder_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dpad_lr_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxBroker>());
  rclcpp::shutdown();
  return 0;
}
