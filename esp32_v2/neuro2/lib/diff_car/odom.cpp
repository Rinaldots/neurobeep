#include "diff_car.h"

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