#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <leg_detector/leg_detector.h>

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
    // Subscribe to the LiDAR data
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar_topic", 10, std::bind(&ObstacleAvoidanceNode::lidarCallback, this, std::placeholders::_1));

    // Create a publisher to control the robot
    robot_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize the leg detector
    leg_detector_ = std::make_shared<leg_detector::LegDetector>();
    leg_detector_->init("leg_detector");
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Analyze the LiDAR data to detect obstacles
    bool obstacle_detected = checkObstacles(msg);

    // If an obstacle is detected, take avoiding action
    if (obstacle_detected) {
      // Implement obstacle avoidance logic here

      // Check if there are any legs in front of the robot
      leg_detector::LegArray::SharedPtr leg_array = leg_detector_->detect(msg);
      if (!leg_array->legs.empty()) {
        // Calculate the angle to the closest leg
        float closest_leg_angle = atan2(leg_array->legs.front().y, leg_array->legs.front().x);

        // Calculate the desired robot movement based on the leg angle
        float angular_velocity = closest_leg_angle * 0.1; // Adjust the scaling factor

        // Publish the Twist message to control the robot's movement
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.2; // Adjust the linear velocity
        cmd_vel.angular.z = angular_velocity;
        robot_publisher_->publish(cmd_vel);
      } else {
        // If there are no legs in front of the robot, stop the robot
        geometry_msgs::msg::Twist stop_twist;
        robot_publisher_->publish(stop_twist);
      }
    }
  }

  std::shared_ptr<leg_detector::LegDetector> leg_detector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}