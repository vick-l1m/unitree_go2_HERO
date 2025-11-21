#include <memory>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cstdint>

using std::placeholders::_1;

class FlattenNode : public rclcpp::Node
{
public:
  FlattenNode()
  : Node("flatten_pointcloud_to_map2d_node")
  {
    // --- declare parameters ---
    this->declare_parameter<double>("resolution", 0.05);
    this->declare_parameter<double>("max_width_m", 200.0);
    this->declare_parameter<double>("max_height_m", 200.0);
    this->declare_parameter<double>("ground_height_threshold", 0.55);
    this->declare_parameter<std::string>("cloud_topic", "/map");
    this->declare_parameter<std::string>("odom_topic", "/lio_sam/mapping/odometry");
    this->declare_parameter<std::string>("map2d_topic", "/map2d");
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<double>("update_radius_m", 15.0);
    this->declare_parameter<double>("tick_rate_hz", 0.5);

    // --- read parameters ---
    this->get_parameter("resolution", resolution_);
    this->get_parameter("max_width_m", max_width_m_);
    this->get_parameter("max_height_m", max_height_m_);
    this->get_parameter("ground_height_threshold", ground_height_thresh_);
    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("map2d_topic", map2d_topic_);
    this->get_parameter("queue_size", queue_size_);
    this->get_parameter("update_radius_m", update_radius_m_);
    this->get_parameter("tick_rate_hz", tick_rate_hz_);

    // --- compute grid dims & origin ---
    global_w_ = static_cast<size_t>(max_width_m_ / resolution_) + 1;
    global_h_ = static_cast<size_t>(max_height_m_ / resolution_) + 1;
    origin_x_ = -0.5 * global_w_ * resolution_;
    origin_y_ = -0.5 * global_h_ * resolution_;

    // --- subscriptions & publisher with QoS matching ---
    // Use BEST_EFFORT to match LIO-SAM's QoS
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_))
                       .best_effort()
                       .durability_volatile();
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sub_qos,
      std::bind(&FlattenNode::odomCallback, this, _1));
    
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sub_qos,
      std::bind(&FlattenNode::cloudCallback, this, _1));
    
    // Publisher with transient_local and reliable for map
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .transient_local()
                       .reliable();
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map2d_topic_, pub_qos);

    // --- init grid & publish once ---
    loadGrid();
    publishFullGrid();

    // --- timer driven by tick_rate_hz_ ---
    auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FlattenNode::update, this));
    
    RCLCPP_INFO(this->get_logger(), "Flatten node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to cloud: %s", cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to odom: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing map2d to: %s", map2d_topic_.c_str());
  }

private:
  void loadGrid()
  {
    grid_.assign(global_w_ * global_h_, -1);
  }

  void publishFullGrid()
  {
    nav_msgs::msg::OccupancyGrid og;
    og.header.stamp    = current_pose_stamp_;
    og.header.frame_id = "map";
    og.info.resolution = resolution_;
    og.info.width      = global_w_;
    og.info.height     = global_h_;
    og.info.origin.position.x    = origin_x_;
    og.info.origin.position.y    = origin_y_;
    og.info.origin.orientation.w = 1.0;
    og.data = grid_;
    publisher_->publish(og);
  }

  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_       = msg->pose.pose;
    current_pose_stamp_ = msg->header.stamp;
    have_pose_ = true;
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    cloud_msg_  = msg;
    have_cloud_ = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                         "Received point cloud with %d points", 
                         msg->width * msg->height);
  }

  void update()
  {
    if (!have_pose_ || !have_cloud_) {
      if (!have_pose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Waiting for odometry data...");
      }
      if (!have_cloud_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Waiting for point cloud data...");
      }
      return;
    }

    // center & radius
    const double cx = current_pose_.position.x;
    const double cy = current_pose_.position.y;
    const double r2 = update_radius_m_ * update_radius_m_;

    // clear local patch
    int ci = int((cx - origin_x_) / resolution_);
    int cj = int((cy - origin_y_) / resolution_);
    int ri = int(update_radius_m_ / resolution_);
    for (int jj = std::max(cj - ri, 0);
         jj <= std::min(cj + ri, int(global_h_) - 1); ++jj)
    {
      for (int ii = std::max(ci - ri, 0);
           ii <= std::min(ci + ri, int(global_w_) - 1); ++ii)
      {
        double wx = origin_x_ + (ii + 0.5) * resolution_;
        double wy = origin_y_ + (jj + 0.5) * resolution_;
        if ((wx - cx)*(wx - cx) + (wy - cy)*(wy - cy) <= r2) {
          grid_[jj * global_w_ + ii] = 0;
        }
      }
    }

    // mark obstacles from pointcloud
    int obstacle_count = 0;
    for (auto it_x = sensor_msgs::PointCloud2Iterator<float>(*cloud_msg_, "x"),
              it_y = sensor_msgs::PointCloud2Iterator<float>(*cloud_msg_, "y"),
              it_z = sensor_msgs::PointCloud2Iterator<float>(*cloud_msg_, "z");
         it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      float px = *it_x, py = *it_y, pz = *it_z;
      if (!std::isfinite(px)||!std::isfinite(py)||!std::isfinite(pz)) continue;
      double dx = px - cx, dy = py - cy;
      if (dx*dx + dy*dy > r2) continue;
      if (pz <= ground_height_thresh_) continue;
      int ix = int((px - origin_x_) / resolution_);
      int iy = int((py - origin_y_) / resolution_);
      if (ix>=0 && ix<int(global_w_) && iy>=0 && iy<int(global_h_)) {
        grid_[iy * global_w_ + ix] = 100;
        obstacle_count++;
      }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Updated map with %d obstacles at position (%.2f, %.2f)",
                         obstacle_count, cx, cy);

    publishFullGrid();
    have_cloud_ = false;  // only process each cloud once
  }

  // parameters
  double resolution_{0.05}, max_width_m_{20.0}, max_height_m_{20.0}, ground_height_thresh_{0.1};
  std::string cloud_topic_{"/points"}, odom_topic_{"/odom"}, map2d_topic_{"/map2d"};
  int    queue_size_{10};
  double update_radius_m_{5.0};
  double tick_rate_hz_{10.0};

  // grid
  size_t global_w_{0}, global_h_{0};
  double origin_x_{0}, origin_y_{0};
  std::vector<int8_t> grid_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::Time current_pose_stamp_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;
  bool have_pose_{false}, have_cloud_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlattenNode>());
  rclcpp::shutdown();
  return 0;
}