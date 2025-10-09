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
#include <ESP32Encoder.h>

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
    unsigned long last_update_raw;
    Pose pose_raw;
    Twist vel_raw;
    Pose pose;
    Twist vel;
    void update_odometry(float v_x_est, float v_y_est, float w_est, float wheel_base);
    void set_angular_position(float roll, float pitch, float yaw);
    void set_position(float x, float y, float z);
    void debug();
    void update_raw_velocity(float left_velocity, float right_velocity, float wheel_base);
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

    #ifdef ENCODER_SIMPLE
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
    #endif

    #ifdef ENCODER_QUAD
    float left_freq_filtered = 0.0;
    float right_freq_filtered = 0.0;
    unsigned long last_sample_time_ms = 0;
    long last_count_left_snapshot = 0;
    long last_count_right_snapshot = 0;
    #endif

    // Velocidade em m/s
    float left_velocity_ms = 0.0;
    float right_velocity_ms = 0.0;
    float left_velocity_ms_abs = 0.0;
    float right_velocity_ms_abs = 0.0;
    float velocity_x_est = 0.0;
    float velocity_y_est = 0.0;
    float angular_velocity_est = 0.0;

    // Velocidade alvo em m/s
    float left_velocity_target = 0.0;
    float right_velocity_target = 0.0;
    float left_velocity_target_abs = 0.0;
    float right_velocity_target_abs = 0.0;

    // Ganhos do PID
    float left_gain = 0.0;
    float right_gain = 0.0;

    bool left_stopped = true;
    bool right_stopped = true;
    uint16_t line_sensor_array[8];
    uint16_t line_position_value = 0;
    int16_t line_distance_mm = 0;  // Distância da linha do centro em mm (inteiro)
    
    String rfid_uid;
    Pose light_sensor;
    Accel mpu_accel;
    bool mpu_available = false; 
    ekf_t ekf;

    // GPS data
    float gps_latitude = 0.0;
    float gps_longitude = 0.0;
    float gps_altitude = 0.0;
    float gps_speed = 0.0;
    bool gps_valid = false;

    void setup();

    // Setup functions
    void setup_encoder_hall();
    void setup_encoder();
    void setup_h_bridge();
    void setup_line_sensor();
    void setup_mpu();
    void setup_gps();
    void setup_rfid();

    // H-Bridge functions
    void update_h_bridge();
    void set_motor_speed(float left_motor_pwm, float right_motor_pwm);
    void set_motor_speed_ms(float vel_left, float vel_right);
    void handler_motor();

    // IMU functions
    void update_mpu();
    void debug_mpu();
    void correct_imu();    // Correção da transformação IMU -> odom
    void calibrate_imu();  // Calibração do IMU
    void calibrate_gyro(uint8_t loops);  // Calibração específica do giroscópio
    void calibrate_accel(uint8_t loops); // Calibração específica do acelerômetro
    void pid_calibration(uint8_t readAddress, float kP, float kI, uint8_t loops); // Algoritmo PID de calibração

    // RFID functions
    void update_rfid();

    // Line sensor functions
    void update_line_position();
    void debug_line();
    int16_t line_dist_center_mm();  // Versão inteira, mais leve
    void follow_line(float base_speed = 0.3, float kp = 0.001);  // Seguidor de linha com PID

    // Line following state
    bool line_following_enabled = false;
    float line_follow_base_speed = 0.3;
    float line_follow_kp = 0.001;  // Ganho proporcional

    // GPS functions
    void displayInfo();
    void loop_gps();

    // Encoder functions
    void debug_encoder();
    void velocity_update();
    MotorPwmResult set_motor_speed_msr(float vel_left, float vel_right);

    // Kalman filter functions
    void update_kalman_filter();
    void init_kf(ekf_t *ekf);
    void run_model(
        ekf_t *ekf,
        double accel_lin, double accel_ang,
        double fx[EKF_N],
        double F[EKF_N*EKF_N],
        double hx[EKF_M],
        double H[EKF_M*EKF_N]
    );
    void reset_kalman_filter();

};

extern DiffCar diffCar;

#endif