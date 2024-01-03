#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#define WHEEL_RADIUS 0.05
#define WHEEL_BASE_HALF 0.17 / 2
#define TRACK_WIDTH_HALF 0.26969 / 2

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10,
        std::bind(&KinematicModel::wheelSpeedCallback, this,
                  std::placeholders::_1));
    publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void
  wheelSpeedCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    double wz = WHEEL_RADIUS / 4 *
                (-1 / (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) * msg->data[0] +
                 1 / (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) * msg->data[1] +
                 1 / (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) * msg->data[2] -
                 1 / (WHEEL_BASE_HALF + TRACK_WIDTH_HALF) * msg->data[3]);

    double vx = WHEEL_RADIUS / 4 *
                (msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3]);

    double vy = WHEEL_RADIUS / 4 *
                (-msg->data[0] + msg->data[1] - msg->data[2] + msg->data[3]);

    cmd_vel_msg.angular.z = wz;
    cmd_vel_msg.linear.x = vx;
    cmd_vel_msg.linear.y = vy;

    publisher->publish(cmd_vel_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  geometry_msgs::msg::Twist cmd_vel_msg;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
