// Include LinearCar class definition
#include "linear.h"  
#include <cmath>

void LinearCar::setup_h_bridge(){
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
}

void LinearCar::set_motor_speed(int speed){
    this->motor_speed = speed;
}


void LinearCar::update_h_bridge(){
    static unsigned long last_t = 0;
	unsigned long now = micros();
    unsigned long espected_delay = 1000000 / (abs(this->motor_speed) * PULSES_PER_REV); 
    // Converte velocidade em delay entre passos
    if(this->motor_speed > 0 && (now - last_t) >= espected_delay){
        step();
        last_t = now;
    }
}

// Realiza um passo
void LinearCar::step(){
    if(this->passo == 0){
        digitalWrite(MOTOR_IN1, 1);
        digitalWrite(MOTOR_IN2, 0);
        digitalWrite(MOTOR_IN3, 0);
        digitalWrite(MOTOR_IN4, 1);
        Serial.println("Passo 0");
    }
    else if(this->passo == 1){
        digitalWrite(MOTOR_IN1, 0);
        digitalWrite(MOTOR_IN2, 1);
        digitalWrite(MOTOR_IN3, 0);
        digitalWrite(MOTOR_IN4, 1);
        Serial.println("Passo 1");
    }
    else if(this->passo == 2){
        digitalWrite(MOTOR_IN1, 0);
        digitalWrite(MOTOR_IN2, 1);
        digitalWrite(MOTOR_IN3, 1);
        digitalWrite(MOTOR_IN4, 0);
        Serial.println("Passo 2");
    }
    else if(this->passo == 3){
        digitalWrite(MOTOR_IN1, 1);
        digitalWrite(MOTOR_IN2, 0);
        digitalWrite(MOTOR_IN3, 1);
        digitalWrite(MOTOR_IN4, 0);
        Serial.println("Passo 3");
    }
    if(this->direction == true){
        this->passo = this->passo + 1;
        if(this->passo > 3){
        this->passo = 0;
        }
    }
    else{
        this->passo = this->passo - 1;
        if(this->passo < 0){
        this->passo = 3;
        }
    }

}

