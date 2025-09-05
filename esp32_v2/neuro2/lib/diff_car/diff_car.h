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

// QTRSensors, ESP32Encoder, MPU9250 extern declarations
#include <QTRSensors.h>



#ifdef ENCODER_QUAD
    extern ESP32Encoder encoder_left;
    extern ESP32Encoder encoder_right;
#endif


extern QTRSensors qtr;
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
    bool calibrated = false;

    int left_motor_pwm = -1;
    int left_motor_dir = -1;
    int right_motor_pwm = -1;
    int right_motor_dir = -1;

    unsigned long encoder_left = 0;
    unsigned long encoder_right = 0;
    unsigned long encoder_left_time_now = 0;
    unsigned long encoder_left_time_last = 0;
    unsigned long encoder_right_time_now = 0;
    unsigned long encoder_right_time_last = 0;

    float left_velocity = 0.0;
    float right_velocity = 0.0;

    float delta_time_velocity = 0;

    float cache_time_velocity = 0;
    float cache_left_pass = 0;
    float cache_right_pass = 0;

    uint16_t line_sensor_array[8];
    uint16_t line_position_value = 0;
    
    String rfid_uid;
    Pose light_sensor;
    Accel mpu_accel;
    ekf_t ekf;

    
    
    void setup_encoder_hall();
    void setup_encoder();
    void setup_h_bridge();
    void setup_line_sensor();
    void setup_mpu();
    void setup_timer();


    void update_h_bridge();
    void set_motor_speed(int left_motor_pwm, int right_motor_pwm);
    
    void update_kalman_filter();
    
    void update_mpu();
    void debug_mpu();

    void setup_rfid();
    void update_rfid();

    void update_line_position();
    void debug_line();

    void debug_encoder();
    void velocity_update();
};

extern DiffCar diffCar;

#endif