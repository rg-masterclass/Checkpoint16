#include <chrono>
#include <cstddef>
#include <numeric>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define WHEEL_RADIUS 0.05
#define WHEEL_BASE_HALF 0.17 / 2
#define TRACK_WIDTH_HALF 0.26969 / 2

using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {

    trajectory = {
        {0.0, 1.0, -1.0},         {0.0, 1.0, 1.0},           {0.0, 1.0, 1.0},
        {1.570796326, 1.0, -1.0}, {-3.14159265, -1.0, -1.0}, {0.0, -1.0, 1.0},
        {0.0, -1.0, 1.0},         {0.0, -1.0, -1.0},
    };

    odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odomCallback, this, std::placeholders::_1));

    publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    timer = this->create_wall_timer(
        10ms, std::bind(&EightTrajectory::timerCallback, this));

    start = std::chrono::high_resolution_clock::now();
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    phi = yaw;
  }

  void timerCallback() {

    double dphi = trajectory[trajectoryCounter][0] / 6;
    double dx = trajectory[trajectoryCounter][1] / 6;
    double dy = trajectory[trajectoryCounter][2] / 6;

    auto twist = velocity2twist(dphi, dx, dy);
    auto wheels = twist2wheels(twist[0], twist[1], twist[2]);

    wheel_speed_msg->data.assign(wheels.begin(), wheels.end());

    publisher->publish(*wheel_speed_msg);

    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - start);

    if (seconds.count() > 10) {

      wheel_speed_msg->data = {0, 0, 0, 0};
      publisher->publish(*wheel_speed_msg);

      rclcpp::sleep_for(std::chrono::seconds(1));

      start = std::chrono::system_clock::now();
      trajectoryCounter++;
    }

    if (trajectoryCounter == trajectory.size()) {
      this->timer->cancel();
      rclcpp::shutdown();
      return;
    }
  }

  std::vector<double> velocity2twist(double dphi, double dx, double dy) {

    std::vector<double> output = {};

    output.push_back(dphi);
    output.push_back(dx * cos(phi) + dy * sin(phi));
    output.push_back(-dx * sin(phi) + dy * cos(phi));

    return output;
  }

  std::vector<double> twist2wheels(double wz, double vx, double vy) {
    std::vector<double> output = {};

    output.push_back((-wz * (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) + vx - vy) /
                     WHEEL_RADIUS);
    output.push_back((wz * (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) + vx + vy) /
                     WHEEL_RADIUS);
    output.push_back((wz * (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) + vx - vy) /
                     WHEEL_RADIUS);
    output.push_back((-wz * (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) + vx + vy) /
                     WHEEL_RADIUS);

    return output;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;

  rclcpp::TimerBase::SharedPtr timer;

  std_msgs::msg::Float32MultiArray::SharedPtr wheel_speed_msg =
      std::make_shared<std_msgs::msg::Float32MultiArray>();

  std::vector<std::vector<double>> trajectory;
  size_t trajectoryCounter = 0;

  std::chrono::time_point<std::chrono::system_clock> start;

  double phi;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EightTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
