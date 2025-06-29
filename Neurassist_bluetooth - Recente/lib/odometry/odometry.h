

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>


class Odometry
{
    public:
        Odometry();
        void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
        void euler_to_quat(float x, float y, float z, float* q);

        float x_pos_;
        float y_pos_;
        float heading_;
};

extern Odometry odometry; // Instância global da classe Odometry
#endif