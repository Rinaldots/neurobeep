#ifndef DIFF_CAR_H
#define DIFF_CAR_H
#include <Arduino.h>

#define EKF_N 5 // Position x and Angular z
#define EKF_M 5 // 
#define _float_t double

#include "MPU9250.h"
#include "ESP32Encoder.h"
#include <tinyekf.h>
// QTRSensors, ESP32Encoder, MPU9250 extern declarations
#include <QTRSensors.h>
extern QTRSensors qtr;
extern ESP32Encoder encoder_left;
extern ESP32Encoder encoder_right;
extern MPU9250 mpu;

// geometry_msgs from ROS2
struct Point {
    float x, y, z;
};
struct Quaternion {
    float w, x, y, z;
};
struct Vector3{
    float x, y, z;
};
struct Pose {
    Point position;
    Quaternion orientation;
};
struct Twist {
    Vector3 linear;
    Vector3 angular;
};
struct Accel{
    Vector3 linear;
    Vector3 angular;
};
// geometry_msgs from ROS2

class Diff_Odometry {
public:
    unsigned long last_update;
    Pose current_pose;
    Pose pose;
    Twist vel;
    void update_odometry(float left_velocity, float right_velocity, float wheel_base);
    void set_angular_position(float roll, float pitch, float yaw);
    void get_position(float &x, float &y, float &z);
    void get_angular_position(float &roll, float &pitch, float &yaw);
};




class DiffCar {
public:
    DiffCar();

    Diff_Odometry odometry;

    bool has_encoder_hall = false;
    bool has_encoder_velocity = false;
    bool has_h_bridge = false;
    bool has_imu = false;

    int left_encoder_hall_pin_a = -1;
    int left_encoder_hall_pin_b = -1;
    int right_encoder_hall_pin_a = -1;
    int right_encoder_hall_pin_b = -1;

    int left_motor_pwm = -1;
    int left_motor_dir = -1;
    int right_motor_pwm = -1;
    int right_motor_dir = -1;

    Pose light_sensor;
    Accel mpu_accel;

    ekf_t ekf;


    void setup_encoder_hall(int left_encoder_hall_pin_a, int left_encoder_hall_pin_b, int right_encoder_hall_pin_a, int right_encoder_hall_pin_b);
    void setup_h_bridge(int left_motor_pwm, int left_motor_dir, int right_motor_pwm, int right_motor_dir);
    void setup_line_sensor(int sensor_a1, int sensor_a2, int sensor_a3, int sensor_a4, int sensor_a5, int sensor_a6,int sensor_a7, int sensor_a8, int SensorCount);
    void update_kalman_filter();
    void update_mpu();
    void update_line_sensor(uint16_t* array);
    uint16_t line_position(uint16_t* array);
};

#define NOBS 5

#endif