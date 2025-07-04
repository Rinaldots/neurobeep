#ifndef MOTOR_CONTROLE_H
#define MOTOR_CONTROLE_H

#include "config.h"
#include <Arduino.h>
#include <PID_v1.h>


// Variáveis de debounce
const int debounce_delay = 10; // 10 ms de debounce
extern unsigned long motor_last_time, motor_actual_time;
extern float delta_distance; // Variável para armazenar a distância percorrida
class MOTOR {
  public:
    // Pinos do motor
    int PIN_A, PIN_PWM_CHANNEL;
    int PIN_INVERTED; // Pino para inversão de direção (ou -1 se não usado)

    //ariaveis para o encoder
    volatile long current_position = 0, previous_position = 0;
    unsigned long current_time = 0, previous_time = 0;
    volatile unsigned long encoder_interrupt_time = 0;

    // Variaveis para o controle do motor
    double currentRpm = 0;    
    double targetRpm = 0;   
    double pwmOutput = 0;    

    MOTOR(int motor_pin_a, int motor_pin_b, int reverse_pin);
    void pwm(int PWM, bool reverse = false); // Função para definir o PWM do motor
    void stop_motor();
    void calculateCurrentRpm();
}; 

extern MOTOR leftWheel;
extern MOTOR rightWheel;

void setupMotors();

// Function to set target RPM for both motors
void setMotorTargetRpm(float left_rpm, float right_rpm);

// Function to stop both motors
void stopMotors();

void motorSpeed();
// Function to reset PID controllers
void resetPIDControllers();

#endif // MOTOR_CONTROLE_H