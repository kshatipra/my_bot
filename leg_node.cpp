#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <leg_detector/leg_detector.h>

class LegDetectionNode : public rclcpp::Node {
public:
  LegDetectionNode() : Node("leg_detection_node") {
    leg_detector_ = std::make_shared<leg_detector::LegDetector>();
    leg_detector_->init("leg_detector");

    // Initialize ROS subscribers and publishers
    leg_sub_ = this->create_subscription<leg_detector::LegArray>(
        "leg_detector/tracks", 10, std::bind(&LegDetectionNode::legCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void legCallback(const leg_detector::LegArray::SharedPtr leg_array) {
    if (!leg_array->legs.empty()) {
      // Calculate the angle to the closest leg
      float closest_leg_angle = atan2(leg_array->legs.front().y, leg_array->legs.front().x);

      // Calculate the desired robot movement based on the leg angle
      float angular_velocity = closest_leg_angle * 0.1; // Adjust the scaling factor

      // Publish the Twist message to control the robot's movement
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.2; // Adjust the linear velocity
      cmd_vel.angular.z = angular_velocity;
      cmd_vel_pub_->publish(cmd_vel);
    }
  }

  std::shared_ptr<leg_detector::LegDetector> leg_detector_;
  rclcpp::Subscription<leg_detector::LegArray>::SharedPtr leg_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegDetectionNode>());
  rclcpp::shutdown();
  return 0;
}