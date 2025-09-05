// Include DiffCar class definition
#include "diff_car.h"   


void DiffCar::setup_h_bridge(){
    pinMode(MOTOR_EN_A, OUTPUT);
    pinMode(MOTOR_EN_B, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    ledcAttachChannel(MOTOR_EN_A, 30000, 8, 0);
    ledcAttachChannel(MOTOR_EN_B, 30000, 8, 1);
}

void DiffCar::set_motor_speed(int left_motor_pwm, int right_motor_pwm){
    this -> left_motor_dir = 1;
    this -> right_motor_dir = 1; 
    this -> left_motor_pwm = abs(left_motor_pwm);
    this -> right_motor_pwm = abs(right_motor_pwm);
}

void DiffCar::update_h_bridge(){

    if(this->left_motor_pwm != 0){
        if (this->left_motor_dir) {
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, HIGH);
            ledcWrite(MOTOR_EN_A, this->left_motor_pwm);
            Serial.println("left moving forward");
        } else {
            digitalWrite(MOTOR_IN1, HIGH);
            digitalWrite(MOTOR_IN2, LOW);
            ledcWrite(MOTOR_EN_A, this->left_motor_pwm);
            Serial.println("left moving backward");
        }
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_EN_A, 0);
    }

    if(this->right_motor_pwm != 0){
        if (this->right_motor_dir) {
            digitalWrite(MOTOR_IN3, HIGH);
            digitalWrite(MOTOR_IN4, LOW);
            ledcWrite(MOTOR_EN_B, this->right_motor_pwm);
            Serial.println("right moving forward");
        } else {
            digitalWrite(MOTOR_IN3, LOW);
            digitalWrite(MOTOR_IN4, HIGH);
            ledcWrite(MOTOR_EN_B, this->right_motor_pwm);
            Serial.println("right moving backward");
        }
  
    } else {
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, LOW);
        ledcWrite(MOTOR_EN_B, 0);
    }

}
