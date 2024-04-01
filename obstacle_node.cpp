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
      // Stop the robot
      geometry_msgs::msg::Twist stop_twist;
      robot_publisher_->publish(stop_twist);
    } else {
      // If no obstacle detected, continue forward
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.2; // Adjust the linear velocity
      cmd_vel.angular.z = 0.0;
      robot_publisher_->publish(cmd_vel);
    }
  }

  bool checkObstacles(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Iterate through the laser scan data to check for obstacles
    for (float range : msg->ranges) {
      // Check if the range is less than a threshold value
      if (range < obstacle_threshold_) {
        // Obstacle detected
        return true;
      }
    }
    // No obstacles detected
    return false;
  }

  std::shared_ptr<leg_detector::LegDetector> leg_detector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_publisher_;
  float obstacle_threshold_ = 0.5; // Threshold distance for detecting obstacles
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
