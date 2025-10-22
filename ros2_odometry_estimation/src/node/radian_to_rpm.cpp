#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <vesc_msgs/msg/rotor_position.hpp>
#include <deque>
#include <numeric>

#include <chrono>
using namespace std::chrono_literals;

class MotorRpmPublisher : public rclcpp::Node
{
public:
  MotorRpmPublisher()
  : Node("radian_to_rpm"), window_size_(5), rpm_(0)
  {
    // Create a subscriber to the encoder data topic
    encoder_sub_ = this->create_subscription<vesc_msgs::msg::RotorPosition>(
      "sensors/rotor_position", 10, std::bind(&MotorRpmPublisher::encoderCallback, this, std::placeholders::_1));

    // Create a publisher for motor RPM
    motor_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor_rpm", 10);

    // Initialize prev_time with current time
    prev_time = this->now();

    // Create a timer to publish at 50 Hz (20 ms)
    timer_ = this->create_wall_timer(
      20ms, std::bind(&MotorRpmPublisher::publish, this));
  }

private:
  void encoderCallback(const vesc_msgs::msg::RotorPosition::SharedPtr msg)
  {
    // Assuming the encoder data received is in radians
    double angle_radians = msg->position; 
    // RCLCPP_INFO(get_logger(),"encoder reading %f",angle_radians);
    // RCLCPP_INFO(get_logger(),"encoder reading previous %f",prev_encoder_angle_);


    // Get current time
    rclcpp::Time current_time = msg->header.stamp;

    // Calculate time difference since last encoder reading
    rclcpp::Duration delta_time = current_time - prev_time;

    // Convert time duration to seconds
    double time_seconds = delta_time.seconds();
    // RCLCPP_INFO(get_logger(),"time_seconds %f",time_seconds);

    time_deltas_.push_back(time_seconds);
    if (time_deltas_.size() > window_size_)
    {
      time_deltas_.pop_front();
    }

    double smoothed_delta_time = std::accumulate(time_deltas_.begin(), time_deltas_.end(), 0.0) / time_deltas_.size();

    // Calculate angular velocity in radians per second
    double angular_velocity_radians_per_second = 0.0;
    if (smoothed_delta_time > 0.007) {
      // Calculate angular velocity only if time difference is non-zero
      double angle_change_radians = angle_radians - prev_encoder_angle_;
      // RCLCPP_INFO(get_logger(),"angle change radian %f",angle_change_radians);

      if (angle_change_radians > M_PI) {
        angle_change_radians -= 2 * M_PI;
      } else if (angle_change_radians < -M_PI) {
        angle_change_radians += 2 * M_PI;
      }

      angular_velocity_radians_per_second = angle_change_radians / smoothed_delta_time;
      //   RCLCPP_INFO(get_logger(),"angular_velocity_radians_per_second %f",angular_velocity_radians_per_second);

      // int rpm = angular_velocity_radians_per_second * 60.0 / (2 * M_PI);
      // RCLCPP_INFO(get_logger(),"rpm of motor %d",rpm);
    }

    // Convert angular velocity to RPM
    rpm_ = angular_velocity_radians_per_second * 60.0 / (2 * M_PI);
    // RCLCPP_INFO(get_logger(),"rpm of motor %d",rpm_);

    // Publish the RPM
    // auto rpm_msg = std_msgs::msg::Float64();
    // rpm_msg.data = rpm;
    // motor_rpm_pub_->publish(rpm_msg);

    // Update previous encoder angle and time
    prev_encoder_angle_ = angle_radians;
    prev_time = current_time;
  }

  void publish()
  {
    // Publish the RPM
    // RCLCPP_INFO(get_logger(), "Publishing RPM: %d", rpm_);
    auto rpm_msg = std_msgs::msg::Float64();
    rpm_msg.data = rpm_;
    motor_rpm_pub_->publish(rpm_msg);
  }

  rclcpp::Subscription<vesc_msgs::msg::RotorPosition>::SharedPtr encoder_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float prev_encoder_angle_ = 0.0;
  rclcpp::Time prev_time;

  std::deque<double> time_deltas_;
  size_t window_size_;
  int rpm_ ;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorRpmPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include <cmath>
// #include <vesc_msgs/msg/rotor_position.hpp>
// #include <deque>
// #include <numeric>

// #include <chrono>
// using namespace std::chrono_literals;

// class MotorRpmPublisher : public rclcpp::Node
// {
// public:
//   MotorRpmPublisher()
//   : Node("radian_to_rpm"), window_size_(10), rpm_(0)
//   {
//     // Create a subscriber to the encoder data topic
//     encoder_sub_ = this->create_subscription<vesc_msgs::msg::RotorPosition>(
//       "sensors/rotor_position", 10, std::bind(&MotorRpmPublisher::encoderCallback, this, std::placeholders::_1));

//     // Create a publisher for motor RPM
//     motor_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor_rpm", 10);

//     // Initialize prev_time with current time
//     prev_time_ = this->now();

//     // Create a timer to publish at 50 Hz (20 ms)
//     timer_ = this->create_wall_timer(
//       20ms, std::bind(&MotorRpmPublisher::publish, this));
//   }

// private:
//   void encoderCallback(const vesc_msgs::msg::RotorPosition::SharedPtr msg)
//   {
//     // Assuming the encoder data received is in radians
//     double angle_radians = msg->position; 

//     // Get current time
//     rclcpp::Time current_time = msg->header.stamp;

//     // Calculate time difference since last encoder reading
//     rclcpp::Duration delta_time = current_time - prev_time_;

//     // Convert time duration to seconds
//     double time_seconds = delta_time.seconds();

//     // Calculate angular velocity in radians per second
//     double angular_velocity_radians_per_second = 0.0;
//     if (time_seconds > 0.001) {  // Ensure a minimum time delta to avoid spikes
//       double angle_change_radians = angle_radians - prev_encoder_angle_;
//       if (angle_change_radians > M_PI) {
//         angle_change_radians -= 2 * M_PI;
//       } else if (angle_change_radians < -M_PI) {
//         angle_change_radians += 2 * M_PI;
//       }

//       angular_velocity_radians_per_second = angle_change_radians / time_seconds;
//     }

//     // Convert angular velocity to RPM
//     double rpm = angular_velocity_radians_per_second * 60.0 / (2 * M_PI);

//     // Add the current RPM to the deque and maintain the window size
//     rpm_values_.push_back(rpm);
//     if (rpm_values_.size() > window_size_) {
//       rpm_values_.pop_front();
//     }

//     // Calculate the average RPM
//     rpm_ = std::accumulate(rpm_values_.begin(), rpm_values_.end(), 0.0) / rpm_values_.size();
//     RCLCPP_INFO(get_logger(), "RPM of motor: %f", rpm_);

//     // Update previous encoder angle and time
//     prev_encoder_angle_ = angle_radians;
//     prev_time_ = current_time;
//   }

//   void publish()
//   {
//     auto rpm_msg = std_msgs::msg::Float64();
//     rpm_msg.data = rpm_;
//     motor_rpm_pub_->publish(rpm_msg);
//   }

//   rclcpp::Subscription<vesc_msgs::msg::RotorPosition>::SharedPtr encoder_sub_;
//   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rpm_pub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   double prev_encoder_angle_ = 0.0;
//   rclcpp::Time prev_time_;

//   std::deque<double> rpm_values_;
//   size_t window_size_;
//   double rpm_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MotorRpmPublisher>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

