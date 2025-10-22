// #include "odometry_estimation_node.h"

// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <vesc_msgs/msg/vesc_state.hpp>
// #include <vesc_msgs/msg/vesc_state_stamped.hpp>

// #include <chrono>
// #include <vector>
// #include <functional>
// #include <numeric>

// using std::placeholders::_1;
// using namespace std::chrono_literals;
// using vesc_msgs::msg::VescStateStamped;

// // Constructor
// OdometryEstimator::OdometryEstimator() : Node("odometry_publisher")
// {
//   // init vehicle model
//   vehicle_model_ = VehicleModel::createConcreteVehicleModel("DifferentialDrive");

//    // create subscribers
//   // right_wheel_subscriber_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
//   //     "sensors/rotor_position_right", 10,
//   //     std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
//   // left_wheel_subscriber_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
//   //     "sensors/rotor_position_left", 10,
//   //     std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

//   // create subscribers
//   right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
//       "sensors/rotor_position_right", 10,
//       std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
//   left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
//       "sensors/rotor_position_left", 10,
//       std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

//   // create publisher and timer
//   publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
//   timer_ = this->create_wall_timer(100ms, std::bind(&OdometryEstimator::publish, this));
// }

// void OdometryEstimator::handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
// {
//   // auto speed_ =  msg->state.speed;
//   auto rpm_right_motor =  msg->data;
//   // RCLCPP_INFO(get_logger(),"rpm of right motor %f",rpm_right_motor);
//   int rpm_right_wheel = rpm_right_motor/9.6115;
//   RCLCPP_INFO(get_logger(),"rpm of right wheel %d",rpm_right_wheel);
//   rpms_right_.push_back(rpm_right_wheel);
// }

// void OdometryEstimator::handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
// {
//   // auto speed_ =  msg->state.speed;
//   auto rpm_left_motor = -1 * msg->data;
//   // RCLCPP_INFO(get_logger(),"rpm of left motor %f",rpm_
//   int rpm_left_wheel = rpm_left_motor/9.6115;
//   RCLCPP_INFO(get_logger(),"rpm of left wheel %d",rpm_left_wheel);
//   rpms_left_.push_back(rpm_left_wheel);
// }

// void OdometryEstimator::publish()
// {
//   // calculate passed time since last publish
//   TimePoint current_time = Clock::now();
//   std::chrono::duration<double> dt = current_time - previous_time_;
//   // calculate average of received rpm signals
//   int rpm_left_avg =std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
//   int rpm_right_avg =std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();
//   // RCLCPP_INFO(get_logger(),"left wheel rpm avg:  %d",rpm_left_avg);
//   // RCLCPP_INFO(get_logger(),"right wheel rpm avg:  %d",rpm_right_avg);
//   rpms_left_.clear();
//   rpms_right_.clear();
//   // calculate new state based on input
//   VehicleState new_state = vehicle_model_->calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt.count());
//   // create quaternion from yaw angle
//   tf2::Quaternion quat;
//   quat.setRPY(0.0, 0.0, new_state.yaw);
//   // fill message and publish
//   auto message = nav_msgs::msg::Odometry();
//   message.header.stamp = this->get_clock()->now();
//   message.header.frame_id = "odom";
//   message.child_frame_id = "base_footprint";
//   message.pose.pose.position.x = new_state.x;
//   message.pose.pose.position.y = new_state.y;
//   message.pose.pose.orientation.x = quat.x();
//   message.pose.pose.orientation.y = quat.y();
//   message.pose.pose.orientation.z = quat.z();
//   message.pose.pose.orientation.w = quat.w();

//   publisher_->publish(message);
  
//   // update internal state
//   state_ = new_state;
//   previous_time_ = current_time;
// }

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<OdometryEstimator>());
//   rclcpp::shutdown();
//   return 0;
// }






#include "odometry_estimation_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <numeric>
#include <cmath>

using namespace std::chrono_literals;

OdometryEstimator::OdometryEstimator() : Node("odometry_publisher")
{
  // create subscribers
  right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "sensors/rotor_position_right", 30,
      std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
  left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "sensors/rotor_position_left", 30,
      std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

  // imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("sensors/imu_right/raw",10,
  //     std::bind(&OdometryEstimator::handleImuInput, this, std::placeholders::_1));

  // create publisher and timer
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&OdometryEstimator::publish, this));

  previous_time_ = Clock::now();
}

// double OdometryEstimator::handleImuInput(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//   auto linear_acc_x = msg->linear_acceleration.x;
//   RCLCPP_INFO(get_logger(), "linear acceleration of the right IMU %f", linear_acc_x);
//   return linear_acc_x;
// }

void OdometryEstimator::handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
{
  auto rpm_right_motor = msg->data;
  int rpm_right_wheel = rpm_right_motor / 9.6115;
  // RCLCPP_INFO(get_logger(), "rpm of right wheel %d", rpm_right_wheel);
  rpms_right_.push_back(rpm_right_wheel);
}

void OdometryEstimator::handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
{
  auto rpm_left_motor = -1 * msg->data;
  int rpm_left_wheel = rpm_left_motor / 9.6115;
  // RCLCPP_INFO(get_logger(), "rpm of left wheel %d", rpm_left_wheel);
  rpms_left_.push_back(rpm_left_wheel);
}

void OdometryEstimator::publish()
{
  TimePoint current_time = Clock::now();
  std::chrono::duration<double> dt = current_time - previous_time_;

  int rpm_left_avg = std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
  int rpm_right_avg = std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();
  // RCLCPP_INFO(get_logger(), "avg rpm of left wheel %d", rpm_left_avg);
  // RCLCPP_INFO(get_logger(), "avg rpm of right wheel %d", rpm_right_avg);
  rpms_left_.clear();
  rpms_right_.clear();

  VehicleState new_state = calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt.count());

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, new_state.yaw);

  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "odom";
  message.child_frame_id = "base_footprint";
  message.pose.pose.position.x = new_state.x;
  message.pose.pose.position.y = new_state.y;
  message.pose.pose.orientation.x = quat.x();
  message.pose.pose.orientation.y = quat.y();
  message.pose.pose.orientation.z = quat.z();
  message.pose.pose.orientation.w = quat.w();
  
  //To publish translational and rotational velocities
  message.twist.twist.linear.x = new_state.linear_velocity;
  message.twist.twist.angular.z = new_state.angular_velocity;

  publisher_->publish(message);

  state_ = new_state;
  previous_time_ = current_time;
}

VehicleState OdometryEstimator::calculateNextState(int rpm_left_wheel, int rpm_right_wheel, VehicleState prev_state, double dt)
{
  VehicleState new_state{};
  
  double v_l = static_cast<double>(rpm_left_wheel) / 60 * 2 * WHEEL_RADIUS * M_PI;
  double v_r = static_cast<double>(rpm_right_wheel) / 60 * 2 * WHEEL_RADIUS * M_PI;
  
  double angular_vel = (v_r - v_l) / VEHICLE_TRACK;
  double linear_vel = (v_l + v_r) / 2;
  double d_l = v_l * dt;
  double d_r = v_r * dt;
  double d = (d_l + d_r) / 2;
  double dyaw = (d_r - d_l) / VEHICLE_TRACK;

  new_state.x = prev_state.x + d * std::cos(prev_state.yaw + dyaw / 2);
  new_state.y = prev_state.y + d * std::sin(prev_state.yaw + dyaw / 2);
  new_state.yaw = wrapAngle(prev_state.yaw + dyaw);
  
  //To publish translational and rotational velocities
  new_state.linear_velocity = linear_vel;
  new_state.angular_velocity = angular_vel;

  return new_state;
}

double OdometryEstimator::wrapAngle(double angle)
{
  while (angle >= 2 * M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryEstimator>());
  rclcpp::shutdown();
  return 0;
}









// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <deque>
// #include <numeric>
// #include <cmath>
// using namespace std::chrono_literals;

// class OdometryEstimator : public rclcpp::Node
// {
// public:
//   OdometryEstimator()
//   : Node("odometry_publisher"), wheel_radius_(0.05), vehicle_track_(0.2), window_size_(5)
//   {
//     right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
//         "sensors/rotor_position_right", 10,
//         std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
//     left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
//         "sensors/rotor_position_left", 10,
//         std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

//     publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
//     timer_ = this->create_wall_timer(100ms, std::bind(&OdometryEstimator::publish, this));

//     previous_time_ = this->now();
//   }

// private:
//   void handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
//   {
//     addRpmToBuffer(rpms_right_, msg->data);
//   }

//   void handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
//   {
//     addRpmToBuffer(rpms_left_, -msg->data);
//   }

//   void addRpmToBuffer(std::deque<double> &buffer, double rpm)
//   {
//     buffer.push_back(rpm);
//     if (buffer.size() > window_size_) {
//       buffer.pop_front();
//     }
//   }

//   double getAverageRpm(const std::deque<double> &buffer)
//   {
//     return std::accumulate(buffer.begin(), buffer.end(), 0.0) / buffer.size();
//   }

//   void publish()
//   {
//     auto current_time = this->now();
//     auto dt = (current_time - previous_time_).seconds();

//     double rpm_left_avg = getAverageRpm(rpms_left_) / 9.6115;
//     double rpm_right_avg = getAverageRpm(rpms_right_) / 9.6115;

//     VehicleState new_state = calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt);

//     tf2::Quaternion quat;
//     quat.setRPY(0.0, 0.0, new_state.yaw);

//     auto message = nav_msgs::msg::Odometry();
//     message.header.stamp = current_time;
//     message.header.frame_id = "odom";
//     message.child_frame_id = "base_footprint";
//     message.pose.pose.position.x = new_state.x;
//     message.pose.pose.position.y = new_state.y;
//     message.pose.pose.orientation.x = quat.x();
//     message.pose.pose.orientation.y = quat.y();
//     message.pose.pose.orientation.z = quat.z();
//     message.pose.pose.orientation.w = quat.w();

//     publisher_->publish(message);

//     state_ = new_state;
//     previous_time_ = current_time;
//   }

//   struct VehicleState
//   {
//     double x = 0.0;
//     double y = 0.0;
//     double yaw = 0.0;
//   };

//   VehicleState calculateNextState(double rpm_left_wheel, double rpm_right_wheel, VehicleState prev_state, double dt)
//   {
//     double v_l = (rpm_left_wheel / 60.0) * 2 * wheel_radius_ * M_PI;
//     double v_r = (rpm_right_wheel / 60.0) * 2 * wheel_radius_ * M_PI;

//     double angular_vel = (v_r - v_l) / vehicle_track_;
//     double linear_vel = (v_l + v_r) / 2.0;
//     double d = linear_vel * dt;
//     double dyaw = angular_vel * dt;

//     VehicleState new_state;
//     new_state.x = prev_state.x + d * std::cos(prev_state.yaw + dyaw / 2);
//     new_state.y = prev_state.y + d * std::sin(prev_state.yaw + dyaw / 2);
//     new_state.yaw = wrapAngle(prev_state.yaw + dyaw);
//     return new_state;
//   }

//   double wrapAngle(double angle)
//   {
//     while (angle >= 2 * M_PI) {
//       angle -= 2 * M_PI;
//     }
//     while (angle < 0) {
//       angle += 2 * M_PI;
//     }
//     return angle;
//   }

//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_subscriber_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_subscriber_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   double wheel_radius_;
//   double vehicle_track_;
//   size_t window_size_;
//   VehicleState state_;
//   rclcpp::Time previous_time_;
//   std::deque<double> rpms_right_;
//   std::deque<double> rpms_left_;
// };

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<OdometryEstimator>());
//   rclcpp::shutdown();
//   return 0;
// }









// #include "odometry_estimation_node.h"

// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <vesc_msgs/msg/vesc_state.hpp>
// #include <vesc_msgs/msg/vesc_state_stamped.hpp>
// #include <sensor_msgs/msg/imu.hpp>

// #include <chrono>
// #include <vector>
// #include <functional>
// #include <numeric>

// using std::placeholders::_1;
// using namespace std::chrono_literals;
// using vesc_msgs::msg::VescStateStamped;

// OdometryEstimator::OdometryEstimator() : Node("odometry_publisher"), state_{0.0, 0.0, 0.0}
// {
//   vehicle_model_ = VehicleModel::createConcreteVehicleModel("DifferentialDrive");

//   right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
//       "sensors/rotor_position_right", 10,
//       std::bind(&OdometryEstimator::handleRightWheelInput, this, _1));
//   left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
//       "sensors/rotor_position_left", 10,
//       std::bind(&OdometryEstimator::handleLeftWheelInput, this, _1));

//   imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
//       "sensors/imu", 10,
//       std::bind(&OdometryEstimator::handleImuInput, this, _1));

//   publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
//   timer_ = this->create_wall_timer(100ms, std::bind(&OdometryEstimator::publish, this));

//   previous_time_ = Clock::now();
// }

// void OdometryEstimator::handleRightWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
// {
//   double rpm_right_motor = msg->data;
//   double rpm_right_wheel = rpm_right_motor / 9.6115;  // Convert motor RPM to wheel RPM
//   rpms_right_.push_back(rpm_right_wheel);
//   if (rpms_right_.size() > window_size_) {
//     rpms_right_.pop_front();
//   }
// }

// void OdometryEstimator::handleLeftWheelInput(const std_msgs::msg::Float64::SharedPtr msg)
// {
//   double rpm_left_motor = -1 * msg->data;
//   double rpm_left_wheel = rpm_left_motor / 9.6115;  // Convert motor RPM to wheel RPM
//   rpms_left_.push_back(rpm_left_wheel);
//   if (rpms_left_.size() > window_size_) {
//     rpms_left_.pop_front();
//   }
// }

// void OdometryEstimator::handleImuInput(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//   imu_data_ = *msg;
// }

// void OdometryEstimator::publish()
// {
//   auto current_time = Clock::now();
//   std::chrono::duration<double> dt = current_time - previous_time_;

//   double rpm_left_avg = std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
//   double rpm_right_avg = std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();

//   VehicleState new_state = vehicle_model_->calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt.count());

//   tf2::Quaternion quat;
//   quat.setRPY(0.0, 0.0, new_state.yaw);

//   auto message = nav_msgs::msg::Odometry();
//   message.header.stamp = this->get_clock()->now();
//   message.header.frame_id = "odom";
//   message.child_frame_id = "base_footprint";
//   message.pose.pose.position.x = new_state.x;
//   message.pose.pose.position.y = new_state.y;
//   message.pose.pose.orientation.x = quat.x();
//   message.pose.pose.orientation.y = quat.y();
//   message.pose.pose.orientation.z = quat.z();
//   message.pose.pose.orientation.w = quat.w();

//   message.twist.twist.angular.x = imu_data_.angular_velocity.x;
//   message.twist.twist.angular.y = imu_data_.angular_velocity.y;
//   message.twist.twist.angular.z = imu_data_.angular_velocity.z;
//   message.twist.twist.linear.x = imu_data_.linear_acceleration.x;
//   message.twist.twist.linear.y = imu_data_.linear_acceleration.y;
//   message.twist.twist.linear.z = imu_data_.linear_acceleration.z;

//   publisher_->publish(message);

//   state_ = new_state;
//   previous_time_ = current_time;
// }

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<OdometryEstimator>());
//   rclcpp::shutdown();
//   return 0;
// }
