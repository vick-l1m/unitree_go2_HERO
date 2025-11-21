#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class ScanPublisher : public rclcpp::Node
{
public:
  ScanPublisher()
  : Node("scan_publisher_node"), count_(0)
  {
    // Declare parameters
    this->declare_parameter("topic", "/scan");
    this->declare_parameter("frame_id", "laser_frame");
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("angle_min", -M_PI);
    this->declare_parameter("angle_max", M_PI);
    this->declare_parameter("angle_increment", M_PI / 180.0);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 8.0);
    
    // Get parameters
    std::string topic = this->get_parameter("topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    
    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);
    
    // Create timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ScanPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), 
                "Scan Publisher Node started. Publishing to: %s at %.1f Hz", 
                topic.c_str(), publish_rate);
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::LaserScan();
    
    // Set header
    message.header.stamp = this->now();
    message.header.frame_id = frame_id_;
    
    // Set scan parameters
    message.angle_min = angle_min_;
    message.angle_max = angle_max_;
    message.angle_increment = angle_increment_;
    message.time_increment = 0.0;
    message.scan_time = 0.1;
    message.range_min = range_min_;
    message.range_max = range_max_;
    
    // Calculate number of ranges
    int num_ranges = static_cast<int>((angle_max_ - angle_min_) / angle_increment_);
    
    // Generate simulated scan data (a simple pattern)
    message.ranges.resize(num_ranges);
    message.intensities.resize(num_ranges);
    
    for (int i = 0; i < num_ranges; ++i) {
      double angle = angle_min_ + i * angle_increment_;
      
      // Create a simple pattern: closer at front and back, farther at sides
      double range = 3.0 + 2.0 * std::sin(angle * 2.0 + count_ * 0.1);
      
      // Add some variation
      range = std::max(range_min_, std::min(range_max_, range));
      
      message.ranges[i] = range;
      message.intensities[i] = 100.0;
    }
    
    // Publish
    publisher_->publish(message);
    
    if (count_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(), "Publishing scan %zu with %d ranges", 
                  count_, num_ranges);
    }
    
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::string frame_id_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_min_;
  double range_max_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanPublisher>();
  
  RCLCPP_INFO(node->get_logger(), "Starting scan publisher node (for testing)...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}