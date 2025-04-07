#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;


class ControlTurtlebot : public rclcpp::Node {
public:

  explicit ControlTurtlebot(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("control_turtlebot", node_options) {

    // Initialize the MutuallyExclusive callback group object
    callback_publisher_burger_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_publisher_waffle_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_cmdvel_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_subscriber_cmdvel_group_;

    burger_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("burger/cmd_vel", 10);

    waffle_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("waffle/cmd_vel", 10);

    burger_timer_ = this->create_wall_timer(
        50ms, std::bind(&ControlTurtlebot::burger_timer_callback, this),
        callback_publisher_burger_group_);
    
    waffle_timer_ = this->create_wall_timer(
        50ms, std::bind(&ControlTurtlebot::waffle_timer_callback, this),
        callback_publisher_waffle_group_); // 

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&ControlTurtlebot::topic_callback, this, std::placeholders::_1),
        options1);
  }


private:
  void burger_timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vx;
    message.angular.z = vz;
    burger_publisher_->publish(message);
  }

  void waffle_timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vx;
    message.angular.z = vz;
    waffle_publisher_->publish(message);
  }

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx = msg->linear.x;
        vz = msg->angular.z;
  }
  

  rclcpp::TimerBase::SharedPtr burger_timer_;
  rclcpp::TimerBase::SharedPtr waffle_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr burger_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr waffle_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  float vx;
  float vz;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_cmdvel_group_;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_burger_group_;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_waffle_group_;
};



/************************************
************** MAIN *****************
*************************************/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ControlTurtlebot> control_turtlebots =
      std::make_shared<ControlTurtlebot>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(control_turtlebots);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}