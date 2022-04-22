#include "geometry_msgs/msg/twist.hpp"    // Twist
#include "rclcpp/rclcpp.hpp"              // ROS Core Libraries
#include "sensor_msgs/msg/laser_scan.hpp" // Laser Scan

using std::placeholders::_1;

class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance() : Node("ObstacleAvoidance") {

    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", sensor_qos,
        std::bind(&ObstacleAvoidance::topic_callback, this, _1));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
    // 200 readings, from right to left, from -57 to 57 degress
    // calculate new velocity cmd
    float min = 10;
    for (int i = 0; i < 1000; i++) { float current = _msg->ranges[i];
      if (current < min) { min = current; } } auto message = this->calculateVelMsg(min);
    publisher_->publish(message);
  }
  geometry_msgs::msg::Twist calculateVelMsg(float distance) {
    auto msg = geometry_msgs::msg::Twist();
    // logic
    RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);
    if (distance <= 1.0) {
      // turn around
      msg.linear.x = 0;
      msg.angular.z = 0.05;
    } else {
      // go straight ahead
      msg.linear.x = 0.2;
      msg.angular.z = 0;
    }
    return msg;
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}