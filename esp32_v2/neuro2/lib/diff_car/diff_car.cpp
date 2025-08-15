
#include "diff_car.h"
QTRSensors qtr;
ESP32Encoder encoder_left;
ESP32Encoder encoder_right;
MPU9250 mpu;

DiffCar::DiffCar() {}


void Diff_Odometry::update_odometry(float left_velocity, float right_velocity, float wheel_base){
    float dt = (millis() - this->last_update) / 1000.0;  // Time step in seconds
    this->last_update = millis();

    float distance_left = left_velocity * dt;
    float distance_right = right_velocity * dt;
    float delta_distance = distance_right - distance_left;
    float delta_theta = (distance_left + distance_right) / wheel_base;
    this->vel.linear.x = (left_velocity + right_velocity) / 2.0;
    this->vel.angular.z = (right_velocity - left_velocity) / 2.0;

    this->pose.position.x += distance_left * cos(this->pose.orientation.z);
    this->pose.position.y += distance_left * sin(this->pose.orientation.z);
    this->pose.orientation.z += delta_theta;
}

void Diff_Odometry::set_angular_position(float roll, float pitch, float yaw){
    this->pose.orientation.x = roll;
    this->pose.orientation.y = pitch;
    this->pose.orientation.z = yaw;
}

void Diff_Odometry::get_position(float &x, float &y, float &z){
    x = this->pose.position.x;
    y = this->pose.position.y;
    z = this->pose.position.z;
}

void Diff_Odometry::get_angular_position(float &roll, float &pitch, float &yaw){
    roll = this->pose.orientation.x;
    pitch = this->pose.orientation.y;
    yaw = this->pose.orientation.z;
}





void DiffCar::setup_encoder_hall(int left_encoder_hall_pin_a, int left_encoder_hall_pin_b, int right_encoder_hall_pin_a, int right_encoder_hall_pin_b){
    this->left_encoder_hall_pin_a = left_encoder_hall_pin_a;
    this->left_encoder_hall_pin_b = left_encoder_hall_pin_b;
    this->right_encoder_hall_pin_a = right_encoder_hall_pin_a;
    this->right_encoder_hall_pin_b = right_encoder_hall_pin_b;
    encoder_left.attachHalfQuad(left_encoder_hall_pin_a, left_encoder_hall_pin_b);
    encoder_right.attachHalfQuad(right_encoder_hall_pin_a, right_encoder_hall_pin_b);
    encoder_left.setCount(0);
    encoder_right.setCount(0);
}

void DiffCar::update_mpu() {
    mpu.update();
    this->mpu_accel.linear.x = mpu.getAccX();
    this->mpu_accel.linear.y = mpu.getAccY();
    this->mpu_accel.linear.z = mpu.getAccZ();
    this->mpu_accel.angular.x = mpu.getGyroX();
    this->mpu_accel.angular.y = mpu.getGyroY();
    this->mpu_accel.angular.z = mpu.getGyroZ();
}

void DiffCar::setup_line_sensor(int sensor_a1, int sensor_a2, int sensor_a3, int sensor_a4, int sensor_a5, int sensor_a6, int sensor_a7, int sensor_a8, int SensorCount){
    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){(uint8_t)sensor_a1, (uint8_t)sensor_a2, (uint8_t)sensor_a3, (uint8_t)sensor_a4, (uint8_t)sensor_a5, (uint8_t)sensor_a6, (uint8_t)sensor_a7, (uint8_t)sensor_a8}, SensorCount);
    

    delay(100);
    Serial.print("Calibrating...");
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    Serial.println(" Done!");
}

void DiffCar::setup_h_bridge(int left_motor_pwm, int left_motor_dir, int right_motor_pwm, int right_motor_dir){
    this->left_motor_pwm = left_motor_pwm;
    this->left_motor_dir = left_motor_dir;
    this->right_motor_pwm = right_motor_pwm;
    this->right_motor_dir = right_motor_dir;
    pinMode(left_motor_pwm, OUTPUT);
    pinMode(left_motor_dir, OUTPUT);
    pinMode(right_motor_pwm, OUTPUT);
    pinMode(right_motor_dir, OUTPUT);
}

void DiffCar::update_line_sensor(uint16_t* array) {
    qtr.readLineBlack(array);
}
uint16_t DiffCar::line_position(uint16_t* array) {
    uint16_t line_position = qtr.readLineBlack(array);
    return line_position;
}
void DiffCar::update_kalman_filter() {
    // Leituras de sensores
    float linear_odom = this->odometry.vel.linear.x;       // m/s
    float angular_odom = this->odometry.vel.angular.z;  // rad/s
    float theta_sensor = this->light_sensor.orientation.z;  // rad
    float accel_lin = this->mpu_accel.linear.x;         // m/s² (IMU)
    float accel_ang = this->mpu_accel.angular.z;        // rad/s² (IMU)

    float z[NOBS] = {linear_odom, angular_odom, theta_sensor};
}