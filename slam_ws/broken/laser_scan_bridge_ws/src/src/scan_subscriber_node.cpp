#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class ScanSubscriber : public rclcpp::Node
{
public:
  ScanSubscriber()
  : Node("scan_subscriber_node")
  {
    // Declare parameters
    this->declare_parameter("input_topic", "/scan");
    this->declare_parameter("output_topic", "/scan_bridge");
    this->declare_parameter("queue_size", 10);
    
    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();
    
    // Create subscriber to receive scan data from Go2 (Foxy)
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, 
      queue_size,
      std::bind(&ScanSubscriber::scan_callback, this, _1));
    
    // Create publisher to republish for nav2 (Humble)
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic, 
      queue_size);
    
    RCLCPP_INFO(this->get_logger(), 
                "Scan Subscriber Node started. Subscribing to: %s, Publishing to: %s", 
                input_topic.c_str(), output_topic.c_str());
    
    // Statistics timer
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ScanSubscriber::print_statistics, this));
    
    message_count_ = 0;
    last_message_time_ = this->now();
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Log first message
    if (message_count_ == 0) {
      RCLCPP_INFO(this->get_logger(), "Received first scan message!");
      RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", msg->header.frame_id.c_str());
      RCLCPP_INFO(this->get_logger(), "  Number of ranges: %zu", msg->ranges.size());
      RCLCPP_INFO(this->get_logger(), "  Angle range: [%.2f, %.2f] rad", 
                  msg->angle_min, msg->angle_max);
      RCLCPP_INFO(this->get_logger(), "  Range limits: [%.2f, %.2f] m", 
                  msg->range_min, msg->range_max);
    }
    
    // Republish the message
    publisher_->publish(*msg);
    
    // Update statistics
    message_count_++;
    last_message_time_ = this->now();
  }
  
  void print_statistics()
  {
    auto time_since_last = (this->now() - last_message_time_).seconds();
    
    if (message_count_ == 0) {
      RCLCPP_WARN(this->get_logger(), "No messages received yet. Check if Go2 is publishing.");
    } else if (time_since_last > 2.0) {
      RCLCPP_WARN(this->get_logger(), 
                  "No new messages for %.1f seconds. Total received: %d", 
                  time_since_last, message_count_);
    } else {
      RCLCPP_INFO(this->get_logger(), 
                  "Bridge is active. Total messages: %d, Last message: %.1f seconds ago", 
                  message_count_, time_since_last);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  int message_count_;
  rclcpp::Time last_message_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanSubscriber>();
  
  RCLCPP_INFO(node->get_logger(), "Starting scan subscriber/bridge node...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}