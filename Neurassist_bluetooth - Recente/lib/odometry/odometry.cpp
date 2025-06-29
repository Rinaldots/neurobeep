#include "odometry.h"
Odometry odometry; // Instância global da classe Odometry

Odometry::Odometry():
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0)
{
}

void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z){
    float delta_heading = angular_vel_z * vel_dt; //radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; //m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //Calcula o quaternion a partir da posiçao central do robô
    //Atualmente não e nescessario
    /*
    float q[4];
    euler_to_quat(0, 0, heading_, q);
    */
}


void Odometry::euler_to_quat(float roll, float pitch, float yaw, float* q) 
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}