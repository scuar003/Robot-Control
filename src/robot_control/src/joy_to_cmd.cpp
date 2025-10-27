#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class JoyToCmd : public rclcpp::Node {
public:
  JoyToCmd()
  : Node("joy_to_cmd")
  {
    // publisher on a timer at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&JoyToCmd::on_timer, this)
    );

    // sub just flags new data
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoyToCmd::on_joy, this, std::placeholders::_1)
    );

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const float lin = - msg->axes[2] * 0.5; 
    const float ang = - msg->axes[3] * 0.5;  
    const float stopMotors = msg->buttons[1];

    // small deadband to ignore noise
    const float deadband = 1e-3f;
    bool is_zero = (std::fabs(lin) < deadband) && (std::fabs(ang) < deadband);

    if (stopMotors == 1.0) {
      latest_twist_.linear.x  =  0.0f;
      latest_twist_.angular.z =  0.0f;
      new_data_ = true;
    }

    // only accept non‑zero or the first zero after motion
    if (!is_zero || (is_zero && !last_is_zero_) && stopMotors == 0.0) {
      last_is_zero_ = is_zero;

      latest_twist_.linear.x  = is_zero ? 0.0f : - lin;
      latest_twist_.angular.z = is_zero ? 0.0f : ang;
      new_data_ = true;
    }
  }

  void on_timer() {
    if (!new_data_) {
      return;  // nothing new to send
    }
    cmd_pub_->publish(latest_twist_);
    new_data_ = false;
  }

  // state
  bool last_is_zero_{true};
  bool new_data_{false};
  geometry_msgs::msg::Twist latest_twist_;

  // ROS objects
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmd>());
  rclcpp::shutdown();
  return 0;
}
