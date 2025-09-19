#ifndef DIFF_CAR_H
#define DIFF_CAR_H

#include "config.cpp"

#include <Arduino.h>
#include "MPU9250.h"
#include <tinyekf.h>
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <QTRSensors.h>
#include <QuickPID.h>

#ifdef ENCODER_QUAD
    extern ESP32Encoder encoder_left;
    extern ESP32Encoder encoder_right;
#endif


extern QTRSensors qtr;
extern MPU9250 mpu;
extern QuickPID left_pid;
extern QuickPID right_pid;
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
struct MotorPwmResult {
    float left;
    float right;
};

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
    bool calibrated = false;

    float left_motor_pwm = -1;
    int left_motor_dir = -1;
    float right_motor_pwm = -1;
    int right_motor_dir = -1;

    volatile long encoder_left_count = 0;
    volatile long encoder_right_count = 0;
    volatile unsigned long left_last_pulse_us = 0;
    volatile unsigned long right_last_pulse_us = 0;
    volatile unsigned long left_last_interval_us = 0;
    volatile unsigned long right_last_interval_us = 0;
    unsigned long left_intervals[MEDIAN_WINDOW] = {0};
    unsigned long right_intervals[MEDIAN_WINDOW] = {0};
    int left_idx = 0;
    int right_idx = 0;
    float left_freq_filtered = 0.0;
    float right_freq_filtered = 0.0;
    unsigned long last_sample_time_ms = 0;
    long last_count_left_snapshot = 0;
    long last_count_right_snapshot = 0;
    float left_velocity_ms = 0.0;
    float right_velocity_ms = 0.0;
    float left_velocity_target = 0.0;
    float right_velocity_target = 0.0;
    

    bool left_stopped = true;
    bool right_stopped = true;
    uint16_t line_sensor_array[8];
    uint16_t line_position_value = 0;
    
    String rfid_uid;
    Pose light_sensor;
    Accel mpu_accel;
    bool mpu_available = false; 
    ekf_t ekf;

    
    
    void setup_encoder_hall();
    void setup_encoder();
    void setup_h_bridge();
    void setup_line_sensor();
    void setup_mpu();
    void setup_timer();


    void update_h_bridge();
    void set_motor_speed(float left_motor_pwm, float right_motor_pwm);
    void set_motor_speed_ms(float vel_left, float vel_right);
    void handler_motor();

    void update_kalman_filter();
    
    void update_mpu();
    void debug_mpu();

    void setup_rfid();
    void update_rfid();

    void update_line_position();
    void debug_line();

    void debug_encoder();
    void velocity_update();
    MotorPwmResult set_motor_speed_msr(float vel_left, float vel_right);
    void handlercounter(unsigned long novoTempo, unsigned long *tempos);
    
};

extern DiffCar diffCar;

#endif