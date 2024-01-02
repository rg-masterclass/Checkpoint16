#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

enum class Movements : char {
  MoveForward,
  MoveBackward,
  MoveSidewaysToTheLeft,
  MoveSidewaysToTheRight,
  TurnClockwise,
  TurnCounterClockwise,
  Stop
};

inline Movements operator++(Movements &d) {
  d = static_cast<Movements>((static_cast<int>(d) + 1) % 7);
  return d;
}

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher") {
    publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
    timer = this->create_wall_timer(
        3s, std::bind(&WheelVelocitiesPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");
  }

private:
  void timerCallback() {
    switch (movement) {
    case Movements::MoveForward:
      msg.data = {10, 10, 10, 10};
      RCLCPP_INFO(this->get_logger(), "Move forward");
      break;

    case Movements::MoveBackward:
      msg.data = {-10, -10, -10, -10};
      RCLCPP_INFO(this->get_logger(), "Move backward");
      break;

    case Movements::MoveSidewaysToTheLeft:
      msg.data = {-10, 10, -10, 10};
      RCLCPP_INFO(this->get_logger(), "Move left");
      break;

    case Movements::MoveSidewaysToTheRight:
      msg.data = {10, -10, 10, -10};
      RCLCPP_INFO(this->get_logger(), "Move right");
      break;

    case Movements::TurnClockwise:
      msg.data = {10, -10, -10, 10};
      RCLCPP_INFO(this->get_logger(), "Turn clockwise");
      break;

    case Movements::TurnCounterClockwise:
      msg.data = {-10, 10, 10, -10};
      RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise");
      break;
    default:
      msg.data = {0, 0, 0, 0};
      RCLCPP_INFO(this->get_logger(), "Stop");
      this->timer->cancel();
      rclcpp::shutdown();
      return;
    }

    publisher->publish(msg);
    ++movement;
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;

  Movements movement = Movements::MoveForward;

  std_msgs::msg::Float32MultiArray msg;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocitiesPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
