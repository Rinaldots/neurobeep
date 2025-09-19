
#include "diff_car.h"

QTRSensors qtr;
MPU9250 mpu;
hw_timer_t * Sensors_timer = NULL;

void IRAM_ATTR sensor_timer(){
    diffCar.update_line_position();
}

DiffCar::DiffCar() {}

void DiffCar::update_kalman_filter() {
    // Leituras de sensores
    float linear_odom = this->odometry.vel.linear.x;       // m/s
    float angular_odom = this->odometry.vel.angular.z;  // rad/s
    float theta_sensor = this->light_sensor.orientation.z;  // rad
    float accel_lin = this->mpu_accel.linear.x;         // m/s² (IMU)
    float accel_ang = this->mpu_accel.angular.z;        // rad/s² (IMU)

    float z[NOBS] = {linear_odom, angular_odom, theta_sensor};
}

void DiffCar::setup_timer() {
    Sensors_timer = timerBegin(20000); 
    timerAttachInterrupt(Sensors_timer, &sensor_timer);
    timerAlarm(Sensors_timer, 10000, true, 0);
}