#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class LineFollower : public rclcpp::Node {
public:
  LineFollower()
  : Node("line_follower")
  , filtered_err_(0.0)
  , new_cmd_(false)
  {
    // parameters
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    this->declare_parameter<double>("linear_speed", 0.4);
    this->declare_parameter<double>("kp", 0.002);
    this->declare_parameter<double>("publish_rate", 2.0);      // Hz
    this->declare_parameter<double>("smoothing_alpha", 0.2);    // [0,1]
    this->declare_parameter<double>("noise_deadband", 5.0);     // pixels

    image_topic_    = this->get_parameter("image_topic").as_string();
    linear_speed_   = this->get_parameter("linear_speed").as_double();
    kp_             = this->get_parameter("kp").as_double();
    double rate     = this->get_parameter("publish_rate").as_double();
    alpha_          = this->get_parameter("smoothing_alpha").as_double();
    noise_deadband_ = this->get_parameter("noise_deadband").as_double();

    // publisher & subscriber
    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 1,
      std::bind(&LineFollower::imageCallback, this, std::placeholders::_1)
    );

    // timer to publish at fixed rate
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LineFollower::publishCmd, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "LineFollower on '%s' @ lin=%.2f, kp=%.4f, pub=%.1fHz, alpha=%.2f, deadband=±%.1fpx",
      image_topic_.c_str(), linear_speed_, kp_, rate, alpha_, noise_deadband_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // convert to BGR
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }
    cv::Mat img = cv_ptr->image;

    // region of interest = bottom half
    cv::Mat roi = img(cv::Rect(0, img.rows/2, img.cols, img.rows/2));

    // HSV threshold for blue
    cv::Mat hsv, mask;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
      cv::Scalar(100, 100, 100),
      cv::Scalar(130, 255, 255),
      mask
    );
    // morphological clean
    cv::erode(mask, mask, {}, cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, {}, cv::Point(-1,-1), 2);

    // find largest contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
      // no line → zero the error
      filtered_err_ = 0.0;
    } else {
      // compute raw centroid error
      size_t best=0; double maxA=0;
      for (size_t i=0; i<contours.size(); ++i) {
        double A = cv::contourArea(contours[i]);
        if (A > maxA) { maxA=A; best=i; }
      }
      cv::Moments M = cv::moments(contours[best]);
      double cx = (M.m00>0) ? (M.m10/M.m00) : (mask.cols/2.0);
      double err = cx - (mask.cols/2.0);

      // exponential smoothing
      filtered_err_ = alpha_*err + (1.0 - alpha_)*filtered_err_;

      // apply deadband
      if (std::fabs(filtered_err_) < noise_deadband_) {
        filtered_err_ = 0.0;
      }
    }

    // prepare latest command
    latest_cmd_.linear.x  = linear_speed_;
    latest_cmd_.angular.z = - kp_ * filtered_err_;
    new_cmd_ = true;
  }

  void publishCmd() {
    if (!new_cmd_) {
      return;
    }
    cmd_pub_->publish(latest_cmd_);
    new_cmd_ = false;
  }

  // ROS objects
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  double            filtered_err_;
  geometry_msgs::msg::Twist latest_cmd_;
  bool              new_cmd_;

  // parameters
  std::string image_topic_;
  double      linear_speed_, kp_;
  double      alpha_, noise_deadband_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollower>());
  rclcpp::shutdown();
  return 0;
}
