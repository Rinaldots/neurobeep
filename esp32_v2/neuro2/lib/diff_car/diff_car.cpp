
#include "diff_car.h"
#include "config.cpp"

QTRSensors qtr;
ESP32Encoder encoder_left;
ESP32Encoder encoder_right;
MPU9250 mpu;
L298NX2 motors(MOTOR_EN_A, MOTOR_IN1_A, MOTOR_IN2_A, MOTOR_EN_B, MOTOR_IN1_B, MOTOR_IN2_B);
DiffCar::DiffCar() {}


#include "odom.cpp"
#include "encoder.cpp"
#include "mpu.cpp"
#include "h_bridge.cpp"
#include "line_sensor.cpp"



void DiffCar::update_kalman_filter() {
    // Leituras de sensores
    float linear_odom = this->odometry.vel.linear.x;       // m/s
    float angular_odom = this->odometry.vel.angular.z;  // rad/s
    float theta_sensor = this->light_sensor.orientation.z;  // rad
    float accel_lin = this->mpu_accel.linear.x;         // m/s² (IMU)
    float accel_ang = this->mpu_accel.angular.z;        // rad/s² (IMU)

    float z[NOBS] = {linear_odom, angular_odom, theta_sensor};
}