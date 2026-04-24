#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "jo_msgs/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

// ── node ─────────────────────────────────────────────────────────────────────

class LidarDynamicFilter : public rclcpp::Node
{
public:
  LidarDynamicFilter()
  : Node("lidar_dynamic_filter"),
    tf_buf_(get_clock()),
    tf_listener_(tf_buf_)
  {
    declare_parameter("tracking_timeout",   1.0);   // s — if lost, pass LiDAR unfiltered
    declare_parameter("filter_margin",      0.2);   // extra radius around obstacle (m)

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points_filtered", 10);

    obs_sub_ = create_subscription<jo_msgs::msg::ObstacleArray>(
      "/onboard_detector/tracked_dynamic_obstacles", 10,
      std::bind(&LidarDynamicFilter::obs_cb, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10,
      std::bind(&LidarDynamicFilter::scan_cb, this, std::placeholders::_1));
  }

private:
  struct ObsSphere
  {
    double x, y;   // obstacle centre in its published frame
    double r;      // bounding sphere radius (half-diagonal + margin)
  };

  void obs_cb(const jo_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_recv_ = now();
    obs_.clear();
    obs_frame_ = msg->header.frame_id;
    const double margin = get_parameter("filter_margin").as_double();
    for (const auto & o : msg->obstacles) {
      obs_.push_back({
        o.pose.position.x,
        o.pose.position.y,
        std::hypot(o.size.x, o.size.y) * 0.5 + margin
      });
    }
  }

  void scan_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Snapshot shared state under lock.
    double timeout;
    bool active;
    std::vector<ObsSphere> local_obs;
    std::string obs_frame;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      timeout   = get_parameter("tracking_timeout").as_double();
      active    = last_recv_.has_value() &&
        (now() - *last_recv_).seconds() < timeout;
      if (active) {
        local_obs = obs_;
        obs_frame = obs_frame_;
      }
    }

    // When tracking is lost pass the cloud through unchanged so that the
    // LiDAR's raytrace clearing naturally decays old costmap marks.
    if (!active || local_obs.empty()) {
      pub_->publish(*msg);
      return;
    }

    // Transform each obstacle centre into the LiDAR sensor frame (cheap:
    // only one point per obstacle, not per LiDAR return).
    const std::string & lidar_frame = msg->header.frame_id;
    std::vector<ObsSphere> obs_lidar;
    obs_lidar.reserve(local_obs.size());
    for (const auto & o : local_obs) {
      geometry_msgs::msg::PointStamped pin, pout;
      pin.header.frame_id = obs_frame;
      // Use time-zero with the node's clock type so tf2 interprets it as
      // "latest available transform" — works correctly with use_sim_time.
      pin.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      pin.point.x = o.x;
      pin.point.y = o.y;
      pin.point.z = 0.0;
      try {
        tf_buf_.transform(pin, pout, lidar_frame, tf2::durationFromSec(0.05));
        obs_lidar.push_back({pout.point.x, pout.point.y, o.r});
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF not ready yet, passing LiDAR unfiltered: %s", ex.what());
        pub_->publish(*msg);
        return;
      }
    }

    // Locate x/y field offsets in the point layout.
    int x_off = -1, y_off = -1;
    for (const auto & f : msg->fields) {
      if (f.name == "x") {x_off = static_cast<int>(f.offset);}
      if (f.name == "y") {y_off = static_cast<int>(f.offset);}
    }
    if (x_off < 0 || y_off < 0) {
      pub_->publish(*msg);
      return;
    }

    // Copy points that do not fall inside any obstacle bounding sphere.
    const uint32_t ps   = msg->point_step;
    const uint32_t n    = msg->width * msg->height;
    const uint8_t * raw = msg->data.data();

    std::vector<uint8_t> kept;
    kept.reserve(n * ps);

    for (uint32_t i = 0; i < n; ++i) {
      const uint8_t * base = raw + i * ps;
      float px, py;
      std::memcpy(&px, base + x_off, sizeof(float));
      std::memcpy(&py, base + y_off, sizeof(float));

      bool inside = false;
      for (const auto & o : obs_lidar) {
        const double dx = px - o.x;
        const double dy = py - o.y;
        if (dx * dx + dy * dy <= o.r * o.r) {
          inside = true;
          break;
        }
      }
      if (!inside) {
        kept.insert(kept.end(), base, base + ps);
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header      = msg->header;
    out.height      = 1;
    out.width       = static_cast<uint32_t>(kept.size() / ps);
    out.fields      = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step  = ps;
    out.row_step    = static_cast<uint32_t>(kept.size());
    out.data        = std::move(kept);
    out.is_dense    = false;
    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  tf2_ros::Buffer   tf_buf_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mtx_;
  std::vector<ObsSphere> obs_;
  std::string obs_frame_;
  std::optional<rclcpp::Time> last_recv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDynamicFilter>());
  rclcpp::shutdown();
}
