// #ifndef ODOMETRY_ESTIMATION_NODE_H
// #define ODOMETRY_ESTIMATION_NODE_H

// #include <chrono>
// #include <nav_msgs/msg/odometry.hpp>
// #include <std_msgs/msg/int64.hpp>
// #include <vector>
// #include <vesc_msgs/msg/vesc_state.hpp>
// #include <vesc_msgs/msg/vesc_state_stamped.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <std_msgs/msg/float64.hpp>


// #include "vehicle_models.h"
// #include "rclcpp/rclcpp.hpp"

// using Clock = std::chrono::high_resolution_clock;
// using TimePoint = std::chrono::time_point<Clock>;
// using vesc_msgs::msg::VescStateStamped;
// using std_msgs::msg::Float32;
// using std_msgs::msg::Float64;

// class OdometryEstimator : public rclcpp::Node {
//  public:
//   OdometryEstimator();

//  private:
//   // void handleRightWheelInput(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
//   // void handleLeftWheelInput(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
//   void handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
//   void handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
//   void publish();
//   VehicleModelPtr vehicle_model_{nullptr};
//   VehicleState state_{0.0, 0.0, 0.0};
//   std::vector<int> rpms_left_;
//   std::vector<int> rpms_right_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
//   // rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_wheel_subscriber_;
//   // rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_wheel_subscriber_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_subscriber_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_subscriber_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   TimePoint previous_time_{};
// };

// #endif  // ODOMETRY_ESTIMATION_NODE_H








#ifndef ODOMETRY_ESTIMATION_NODE_H
#define ODOMETRY_ESTIMATION_NODE_H

#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

constexpr double WHEEL_RADIUS = 0.125;
constexpr double VEHICLE_TRACK = 0.505;

struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  
  //To publish translational and rotational velocities
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;

};

class OdometryEstimator : public rclcpp::Node {
 public:
  OdometryEstimator();

 private:
  void handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
  void handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
  // double handleImuInput(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publish();
  VehicleState calculateNextState(int rpm_left_wheel, int rpm_right_wheel, VehicleState prev_state, double dt);
  double wrapAngle(double angle);

  VehicleState state_{0.0, 0.0, 0.0};
  std::vector<int> rpms_left_;
  std::vector<int> rpms_right_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_subscriber_;
  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  TimePoint previous_time_{};
};

#endif  // ODOMETRY_ESTIMATION_NODE_H




// #ifndef ODOMETRY_ESTIMATION_NODE_H
// #define ODOMETRY_ESTIMATION_NODE_H

// #include <chrono>
// #include <deque>
// #include <nav_msgs/msg/odometry.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <numeric>
// #include <cmath>

// using Clock = std::chrono::high_resolution_clock;
// using TimePoint = std::chrono::time_point<Clock>;

// constexpr double WHEEL_RADIUS = 0.125;
// constexpr double VEHICLE_TRACK = 0.505;

// struct VehicleState {
//   double x = 0.0;
//   double y = 0.0;
//   double yaw = 0.0;
// };

// class OdometryEstimator : public rclcpp::Node {
// public:
//   OdometryEstimator();

// private:
//   void handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
//   void handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
//   void publish();
  
//   void addRpmToBuffer(std::deque<double> &buffer, double rpm);
//   double getAverageRpm(const std::deque<double> &buffer);
//   VehicleState calculateNextState(double rpm_left_wheel, double rpm_right_wheel, VehicleState prev_state, double dt);
//   double wrapAngle(double angle);

//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_subscriber_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_subscriber_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   double wheel_radius_;
//   double vehicle_track_;
//   size_t window_size_;
//   VehicleState state_;
//   rclcpp::Time previous_time_;
//   std::deque<double> rpms_right_;
//   std::deque<double> rpms_left_;
// };

// #endif  // ODOMETRY_ESTIMATION_NODE_H








// #ifndef ODOMETRY_ESTIMATION_NODE_H
// #define ODOMETRY_ESTIMATION_NODE_H

// #include <chrono>
// #include <nav_msgs/msg/odometry.hpp>
// #include <std_msgs/msg/int64.hpp>
// #include <vector>
// #include <deque>
// #include <vesc_msgs/msg/vesc_state.hpp>
// #include <vesc_msgs/msg/vesc_state_stamped.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <sensor_msgs/msg/imu.hpp>

// #include "vehicle_models.h"
// #include "rclcpp/rclcpp.hpp"

// using Clock = std::chrono::high_resolution_clock;
// using TimePoint = std::chrono::time_point<Clock>;
// using vesc_msgs::msg::VescStateStamped;
// using std_msgs::msg::Float32;
// using std_msgs::msg::Float64;

// class OdometryEstimator : public rclcpp::Node {
//  public:
//   OdometryEstimator();

//  private:
//   void handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
//   void handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg);
//   void handleImuInput(const sensor_msgs::msg::Imu::SharedPtr msg);
//   void publish();

//   VehicleModelPtr vehicle_model_{nullptr};
//   VehicleState state_{0.0, 0.0, 0.0};

//   std::deque<double> rpms_left_;
//   std::deque<double> rpms_right_;
//   size_t window_size_ = 10;

//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_subscriber_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_subscriber_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   TimePoint previous_time_;
//   sensor_msgs::msg::Imu imu_data_;
// };

// #endif  // ODOMETRY_ESTIMATION_NODE_H
