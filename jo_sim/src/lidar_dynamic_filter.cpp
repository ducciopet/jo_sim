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

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ── node ─────────────────────────────────────────────────────────────────────

class LidarDynamicFilter : public rclcpp::Node
{
public:
  LidarDynamicFilter()
  : Node("lidar_dynamic_filter"),
    tf_buf_(get_clock()),
    tf_listener_(tf_buf_)
  {
    declare_parameter("tracking_timeout", 1.0);  // s — if lost, pass LiDAR unfiltered
    declare_parameter("filter_margin",    0.1);  // extra half-extent along each axis (m)

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
  // Obstacle stored in its source frame (before TF to lidar frame).
  struct ObsBox
  {
    double x, y, z;        // centre in obs_frame
    double hx, hy, hz;     // half-extents + margin
    double yaw;            // heading in obs_frame
  };

  // Obstacle ready for per-point test, expressed in the lidar sensor frame.
  struct ObsBoxLidar
  {
    double x, y, z;
    double hx, hy, hz;
    double cos_yaw, sin_yaw;
  };

  void obs_cb(const jo_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_recv_ = now();
    obs_.clear();
    obs_frame_ = msg->header.frame_id;
    const double m = get_parameter("filter_margin").as_double();
    for (const auto & o : msg->obstacles) {
      obs_.push_back({
        o.pose.position.x,
        o.pose.position.y,
        o.pose.position.z,
        o.size.x * 0.5 + m,
        o.size.y * 0.5 + m,
        o.size.z * 0.5 + m,
        yaw_from_quat(o.pose.orientation)
      });
    }
  }

  void scan_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    double timeout;
    bool active;
    std::vector<ObsBox> local_obs;
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

    if (!active || local_obs.empty()) {
      pub_->publish(*msg);
      return;
    }

    const std::string & lidar_frame = msg->header.frame_id;
    const rclcpp::Time t0(0, 0, get_clock()->get_clock_type());

    // Look up the transform once to adjust obstacle yaw to lidar frame.
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buf_.lookupTransform(lidar_frame, obs_frame, t0,
                                            tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF not ready yet, passing LiDAR unfiltered: %s", ex.what());
      pub_->publish(*msg);
      return;
    }
    const double tf_yaw = yaw_from_quat(tf_stamped.transform.rotation);

    // Transform each obstacle centre to lidar frame and compute lidar-frame yaw.
    std::vector<ObsBoxLidar> obs_lidar;
    obs_lidar.reserve(local_obs.size());
    for (const auto & o : local_obs) {
      geometry_msgs::msg::PointStamped pin, pout;
      pin.header.frame_id = obs_frame;
      pin.header.stamp    = t0;
      pin.point.x = o.x;
      pin.point.y = o.y;
      pin.point.z = o.z;
      try {
        tf_buf_.transform(pin, pout, lidar_frame, tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF centre transform failed, passing LiDAR unfiltered: %s", ex.what());
        pub_->publish(*msg);
        return;
      }
      const double box_yaw = o.yaw + tf_yaw;
      obs_lidar.push_back({
        pout.point.x, pout.point.y, pout.point.z,
        o.hx, o.hy, o.hz,
        std::cos(box_yaw), std::sin(box_yaw)
      });
    }

    // Locate x/y/z field offsets in the point layout.
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto & f : msg->fields) {
      if (f.name == "x") {x_off = static_cast<int>(f.offset);}
      if (f.name == "y") {y_off = static_cast<int>(f.offset);}
      if (f.name == "z") {z_off = static_cast<int>(f.offset);}
    }
    if (x_off < 0 || y_off < 0) {
      pub_->publish(*msg);
      return;
    }

    // Copy points that do not fall inside any obstacle oriented box.
    const uint32_t ps   = msg->point_step;
    const uint32_t n    = msg->width * msg->height;
    const uint8_t * raw = msg->data.data();

    std::vector<uint8_t> kept;
    kept.reserve(n * ps);

    for (uint32_t i = 0; i < n; ++i) {
      const uint8_t * base = raw + i * ps;
      float px, py, pz = 0.0f;
      std::memcpy(&px, base + x_off, sizeof(float));
      std::memcpy(&py, base + y_off, sizeof(float));
      if (z_off >= 0) {
        std::memcpy(&pz, base + z_off, sizeof(float));
      }

      bool inside = false;
      for (const auto & o : obs_lidar) {
        const double dx = px - o.x;
        const double dy = py - o.y;
        const double dz = pz - o.z;
        // Rotate (dx,dy) into the obstacle-aligned frame.
        const double u =  dx * o.cos_yaw + dy * o.sin_yaw;
        const double v = -dx * o.sin_yaw + dy * o.cos_yaw;
        if (std::abs(u) <= o.hx && std::abs(v) <= o.hy && std::abs(dz) <= o.hz) {
          inside = true;
          break;
        }
      }
      if (!inside) {
        kept.insert(kept.end(), base, base + ps);
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header       = msg->header;
    out.height       = 1;
    out.width        = static_cast<uint32_t>(kept.size() / ps);
    out.fields       = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step   = ps;
    out.row_step     = static_cast<uint32_t>(kept.size());
    out.data         = std::move(kept);
    out.is_dense     = false;
    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  tf2_ros::Buffer            tf_buf_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex             mtx_;
  std::vector<ObsBox>    obs_;
  std::string            obs_frame_;
  std::optional<rclcpp::Time> last_recv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDynamicFilter>());
  rclcpp::shutdown();
}
