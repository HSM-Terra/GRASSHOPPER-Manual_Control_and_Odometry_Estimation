#include "vehicle_models.h"
#include <string>
#include <cmath>
#include <limits>


std::unique_ptr<VehicleModel> VehicleModel::createConcreteVehicleModel(std::string model_name)
{
  if (model_name.compare("DifferentialDrive") == 0) {
    return DifferentialDriveModel::create();
  }
  else {
    return DifferentialDriveModel::create();
  }
}

std::unique_ptr<VehicleModel> DifferentialDriveModel::create()
{
  return VehicleModelPtr{new DifferentialDriveModel};
}

DifferentialDriveModel::DifferentialDriveModel()
{
  // Intentionally blank
}

VehicleState DifferentialDriveModel::calculateNextState(int rpm_left_wheel, int rpm_right_wheel,
                                                        VehicleState prev_state, double dt)
{
  VehicleState new_state{};
  // calculate wheel velocities
  double v_l = static_cast<double>(rpm_left_wheel / 60 ) * 2 * WHEEL_RADIUS * M_PI;
  double v_r = static_cast<double>(rpm_right_wheel / 60 ) * 2 * WHEEL_RADIUS * M_PI;
  // calculate angular velocity
  double angular_vel = (v_r - v_l) / VEHICLE_TRACK;
  double linear_vel = (v_l + v_r)/2 ;
  double d_l = v_l * dt;
  double d_r = v_r * dt;
  double d = (d_l+d_r)/2;
  // double dyaw = angular_vel * dt; 
  double dyaw = (d_r - d_l)/ VEHICLE_TRACK;
  // dyaw = HelperMethods::wrapAngle(dyaw);

  new_state.x= (prev_state.x + d*std::cos(prev_state.yaw + (dyaw/2)));
  new_state.y= (prev_state.y + d*std::sin(prev_state.yaw + (dyaw/2)));
  new_state.yaw = HelperMethods::wrapAngle(prev_state.yaw + dyaw);
  return new_state;
}

double HelperMethods::wrapAngle(double angle)
{
  while (angle >= 2 * M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle;
}





// #include "vehicle_models.h"
// #include <cmath>

// VehicleModelPtr VehicleModel::createConcreteVehicleModel(std::string model_name) {
//   if (model_name == "DifferentialDrive") {
//     return DifferentialDriveModel::create();
//   }
//   // Add other models as needed
//   return nullptr;
// }

// VehicleState DifferentialDriveModel::calculateNextState(int rpm_left_wheel, int rpm_right_wheel,
//                                                         VehicleState prev_state, double dt) {
//   double wheel_circumference = 2 * M_PI * WHEEL_RADIUS;
//   double left_wheel_speed = (rpm_left_wheel / 60.0) * wheel_circumference;
//   double right_wheel_speed = (rpm_right_wheel / 60.0) * wheel_circumference;
//   double linear_velocity = (right_wheel_speed + left_wheel_speed) / 2.0;
//   double angular_velocity = (right_wheel_speed - left_wheel_speed) / VEHICLE_TRACK;

//   double new_x = prev_state.x + linear_velocity * dt * cos(prev_state.yaw);
//   double new_y = prev_state.y + linear_velocity * dt * sin(prev_state.yaw);
//   double new_yaw = prev_state.yaw + angular_velocity * dt;

//   return {new_x, new_y, HelperMethods::wrapAngle(new_yaw)};
// }

// DifferentialDriveModel::DifferentialDriveModel() {}

// VehicleModelPtr DifferentialDriveModel::create() {
//   return std::unique_ptr<DifferentialDriveModel>(new DifferentialDriveModel());
// }

// double HelperMethods::wrapAngle(double angle) {
//   while (angle > M_PI) angle -= 2.0 * M_PI;
//   while (angle < -M_PI) angle += 2.0 * M_PI;
//   return angle;
// }
