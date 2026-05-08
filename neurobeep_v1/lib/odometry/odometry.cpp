#include "odometry.h"
#include "config.h"
Odometry odometry; // Instância global da classe Odometry

void Odometry::update(float linear_vel_r, float linear_vel_l){
    float delta_x = (linear_vel_r + linear_vel_l) / 2.0f; // Média das velocidades lineares
    float delta_y = 0.0f; // Considerando que não há movimento lateral
    float delta_heading = (linear_vel_r - linear_vel_l) / wheels_y_distance_;

    //calculate current position of the robot
    this->x_pos_ += delta_x;
    this->y_pos_ += delta_y;
    this->heading_ += delta_heading;
    
}


