#include "diff_car.h"

void Diff_Odometry::update_odometry(float v_x_est, float v_y_est, float w_est, float wheel_base) {
    // Calcula delta t
    float dt = (millis() - this->last_update) / 1000.0f;  
    this->last_update = millis();

    // Atualização de pose usando velocidades estimadas
    // Usa integração de Runge-Kutta 2ª ordem para reduzir erro
    float theta = this->pose.orientation.z;
    float theta_mid = theta + w_est * dt * 0.5f;

    // Integra posição
    this->pose.position.x += (v_x_est * cos(theta_mid) - v_y_est * sin(theta_mid)) * dt;
    this->pose.position.y += (v_x_est * sin(theta_mid) + v_y_est * cos(theta_mid)) * dt;

    // Integra orientação
    this->pose.orientation.z += w_est * dt;

    // Atualiza velocidades
    this->vel.linear.x = v_x_est;
    this->vel.linear.y = v_y_est;
    this->vel.angular.z = w_est;
}

void Diff_Odometry::update_raw_velocity(float left_velocity, float right_velocity, float wheel_base){
    float dt = (millis() - this->last_update_raw) / 1000.0;  // tempo em segundos
    this->last_update_raw = millis();

    // Distâncias percorridas por cada roda
    float distance_left = left_velocity * dt;
    float distance_right = right_velocity * dt;

    // Velocidades médias
    float v = (distance_right + distance_left) / (2.0 * dt);
    float w = (distance_right - distance_left) / (wheel_base * dt);

    // Atualiza
    this->vel_raw.linear.x = v;
    this->vel_raw.angular.z = w;
}

void Diff_Odometry::set_angular_position(float roll, float pitch, float yaw){
    this->pose.orientation.x = roll;
    this->pose.orientation.y = pitch;
    this->pose.orientation.z = yaw;
}
void Diff_Odometry::set_position(float x, float y, float z){
    this->pose.position.x = x;
    this->pose.position.y = y;
    this->pose.position.z = z;
}

void Diff_Odometry::debug(){
    Serial.print("Position [m]: ");
    Serial.print(this->pose.position.x, 3);
    Serial.print(", ");
    Serial.print(this->pose.position.y, 3);
    Serial.print(", ");
    Serial.print(this->pose.position.z, 3);
    Serial.print("Orientation [rad]: ");
    Serial.print(this->pose.orientation.x, 3);
    Serial.print(", ");
    Serial.print(this->pose.orientation.y, 3);
    Serial.print(", ");
    Serial.print(this->pose.orientation.z, 3);
    Serial.print("\nVelocity [m/s]: ");
    Serial.print(this->vel.linear.x, 3);
    Serial.print(", ");
    Serial.print(this->vel.linear.y, 3);
    Serial.print("Angular Velocity [rad/s]: ");
    Serial.print(this->vel.angular.z, 3);
    Serial.println();
}
