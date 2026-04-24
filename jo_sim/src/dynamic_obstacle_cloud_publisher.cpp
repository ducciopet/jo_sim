#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "jo_msgs/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

// ── helpers ──────────────────────────────────────────────────────────────────

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Fill an asymmetric egg ellipse with (x,y,z) world-frame points.
// Front semi-axis (velocity dir): base_r + sf * speed
// Back  semi-axis               : base_r
// Lateral semi-axis             : base_r
static std::vector<std::array<float, 3>> egg_points(
  double cx, double cy, double heading, double speed,
  double base_r, double sf, double res, double z)
{
  const double a_front = base_r + sf * speed;
  const double a_back  = base_r;
  const double b       = base_r;
  const double cos_h   = std::cos(heading);
  const double sin_h   = std::sin(heading);
  const int    steps   = static_cast<int>(std::max(a_front, b) / res) + 2;

  std::vector<std::array<float, 3>> pts;
  for (int i = -steps; i <= steps; ++i) {
    for (int j = -steps; j <= steps; ++j) {
      const double u = i * res;
      const double v = j * res;
      const double a = (u >= 0.0) ? a_front : a_back;
      if ((u / a) * (u / a) + (v / b) * (v / b) <= 1.0) {
        pts.push_back({
          static_cast<float>(cx + u * cos_h - v * sin_h),
          static_cast<float>(cy + u * sin_h + v * cos_h),
          static_cast<float>(z)
        });
      }
    }
  }
  return pts;
}

static sensor_msgs::msg::PointCloud2 build_cloud(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp    = stamp;
  cloud.header.frame_id = frame_id;
  cloud.height          = 1;
  cloud.width           = static_cast<uint32_t>(points.size());
  cloud.is_bigendian    = false;
  cloud.is_dense        = true;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
  for (const auto & p : points) {
    *ix = p[0]; ++ix;
    *iy = p[1]; ++iy;
    *iz = p[2]; ++iz;
  }
  return cloud;
}

// ── node ─────────────────────────────────────────────────────────────────────

class DynamicObstacleCloudPublisher : public rclcpp::Node
{
public:
  DynamicObstacleCloudPublisher()
  : Node("dynamic_obstacle_cloud_publisher")
  {
    declare_parameter("base_radius",      0.45);   // (m) base half-size + margin
    declare_parameter("forward_scale",    1.2);    // extra length per m/s forward
    declare_parameter("resolution",       0.08);   // grid spacing (m)
    declare_parameter("point_z",          0.5);    // height of cloud points (m)
    declare_parameter("tracking_timeout", 1.0);    // s before clearing costmap

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_obstacles/costmap_cloud", 10);

    sub_ = create_subscription<jo_msgs::msg::ObstacleArray>(
      "/onboard_detector/tracked_dynamic_obstacles", 10,
      std::bind(&DynamicObstacleCloudPublisher::obs_cb, this, std::placeholders::_1));

    watchdog_ = create_wall_timer(
      100ms, std::bind(&DynamicObstacleCloudPublisher::watchdog_cb, this));
  }

private:
  void obs_cb(const jo_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    last_recv_  = now();
    empty_sent_ = false;

    const double base_r = get_parameter("base_radius").as_double();
    const double sf     = get_parameter("forward_scale").as_double();
    const double res    = get_parameter("resolution").as_double();
    const double z      = get_parameter("point_z").as_double();

    std::vector<std::array<float, 3>> all_pts;
    for (const auto & obs : msg->obstacles) {
      const double vx    = obs.twist.linear.x;
      const double vy    = obs.twist.linear.y;
      const double speed = std::hypot(vx, vy);
      const double head  = (speed > 0.05)
        ? std::atan2(vy, vx)
        : yaw_from_quat(obs.pose.orientation);

      auto pts = egg_points(obs.pose.position.x, obs.pose.position.y,
        head, speed, base_r, sf, res, z);
      all_pts.insert(all_pts.end(), pts.begin(), pts.end());
    }

    pub_->publish(build_cloud(msg->header.frame_id,
      rclcpp::Time(msg->header.stamp), all_pts));
  }

  void watchdog_cb()
  {
    if (empty_sent_ || !last_recv_) {return;}
    const double timeout = get_parameter("tracking_timeout").as_double();
    if ((now() - *last_recv_).seconds() > timeout) {
      pub_->publish(build_cloud("odom", now(), {}));
      empty_sent_ = true;
      RCLCPP_INFO(get_logger(), "Tracking lost — costmap cloud cleared.");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::optional<rclcpp::Time> last_recv_;
  bool empty_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacleCloudPublisher>());
  rclcpp::shutdown();
}
