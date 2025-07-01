

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>


class Odometry
{
    public:
        void update(float linear_vel_r, float linear_vel_l);
        float x_pos_;
        float y_pos_;
        float heading_;
};

extern Odometry odometry; // Instância global da classe Odometry
#endif