#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RobotMovementNode : public rclcpp::Node {
public:
  RobotMovementNode() : Node("robot_movement_node") {
    // Create a publisher to control the robot
    robot_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void moveRobot(const std::string& direction) {
    // Define the desired robot movement based on the given direction
    geometry_msgs::msg::Twist cmd_vel;
    if (direction == "forward") {
      cmd_vel.linear.x = 0.2; // Adjust the linear velocity
      cmd_vel.angular.z = 0;
    } else if (direction == "backward") {
      cmd_vel.linear.x = -0.2; // Adjust the linear velocity
      cmd_vel.angular.z = 0;
    } else if (direction == "left") {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0.1; // Adjust the angular velocity
    } else if (direction == "right") {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = -0.1; // Adjust the angular velocity
    }

    // Publish the Twist message to control the robot's movement
    robot_publisher_->publish(cmd_vel);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMovementNode>());
  rclcpp::shutdown();
  return 0;
}