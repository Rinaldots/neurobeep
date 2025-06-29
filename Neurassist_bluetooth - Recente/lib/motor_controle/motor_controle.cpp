#include <Arduino.h>
#include "motor_controle.h"

// Default PID parameters, can be tuned
const float DEFAULT_KP = 2.0f;
const float DEFAULT_KI = 0.4f;
const float DEFAULT_KD = 0.002f;
const float DEFAULT_MIN_PWM = 520.0f; // Corresponds to original left_motor_min/right_motor_min

MOTOR leftWheel(B_IA, B_IB, false, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN_PWM);
MOTOR rightWheel(A_IB, A_IA, false, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN_PWM);

void IRAM_ATTR a_encoder() {
  unsigned long current_time = millis();
  if (current_time - leftWheel.encoder_interrupt_time > debounce_delay) {
    leftWheel.CurrentPosition--;
    leftWheel.encoder_interrupt_time = current_time;
  }
}
void IRAM_ATTR b_encoder() {
  unsigned long current_time = millis();
  if (current_time - rightWheel.encoder_interrupt_time > debounce_delay) {
    rightWheel.CurrentPosition--;
    rightWheel.encoder_interrupt_time = current_time;
  }
}

MOTOR::MOTOR(int8_t motor_pin_a, int8_t motor_pin_b, bool is_inverted, float p_gain, float i_gain, float d_gain, float min_pwm_val) {
  this->PIN_A = motor_pin_a;
  this->PIN_B = motor_pin_b;
  this->inverted = is_inverted;
  
  this->CurrentPosition = 0;
  this->PreviousPosition = 0;
  this->current_time_2 = 0;
  this->previous_time_2 = 0;
  this->encoder_interrupt_time = 0;

  this->kp = p_gain;
  this->ki = i_gain;
  this->kd = d_gain;
  this->minPwm = min_pwm_val;

  this->integral = 0.0f;
  this->previousError = 0.0f;
  this->targetRpm = 0.0;
  this->currentRpm = 0.0;
  this->pwmOutput = 0.0;

  ledcAttach(this->PIN_A, freq, resolution);
  ledcAttach(this->PIN_B, freq, resolution);
}

void MOTOR::pwm(int PWM, bool Direction) {
  if (inverted) {
    if (Direction) {
      ledcWrite(PIN_B, 0);
      ledcWrite(PIN_A, 1023 - PWM);
    } else {
      ledcWrite(PIN_B, 1023 - PWM);
      ledcWrite(PIN_A, 0);
    }
  } else {
    if (Direction) {
      ledcWrite(PIN_A, 0);
      ledcWrite(PIN_B, PWM);
    } else {
      ledcWrite(PIN_A, PWM);
      ledcWrite(PIN_B, 0);
    }
  }
}

void MOTOR::calculateCurrentRpm() {
  current_time_2 = millis();
  long current_encoder_pos = this->CurrentPosition; 
  unsigned long delta_time_ms = (current_time_2 - previous_time_2);

  if (delta_time_ms >= 200) { // Calcula a RPM a cada 50 ms
    long delta_position = abs(current_encoder_pos - PreviousPosition);

    if (delta_time_ms > 0) { // Evita divisão por zero
        this->currentRpm = (static_cast<float>(delta_position) * 60000.0f) / (ENCODER_PULSES_PER_REVOLUTION * delta_time_ms);
    } else {
        this->currentRpm = 0; 
    }
    PreviousPosition = CurrentPosition;
    previous_time_2 = current_time_2;
  }
}

void MOTOR::calculatePid() {
    // If targetRpm is 0, reset PID state and output.
    // This ensures that when the motor is asked to stop, PID terms don't linger.
    if (this->targetRpm == 0) {
        this->pwmOutput = 0;
        this->integral = 0.0f;
        this->previousError = 0.0f;
        return;
    }

    float error = this->targetRpm - this->currentRpm;

    this->integral += error;
    // Previne o termo integral de crescer demais
    if (this->ki != 0) {
        float max_integral_contribution = 1023.0f * 0.4f; // e.g., a integral nao deve contribuir mais que 40% do PWM maximo
        float max_abs_integral = fabsf(this->ki) > 1e-6 ? (max_integral_contribution / fabsf(this->ki)) : 0.0f;
        if (this->integral > max_abs_integral) this->integral = max_abs_integral;
        else if (this->integral < -max_abs_integral) this->integral = -max_abs_integral;
    }

    float derivative = error - this->previousError;
    this->previousError = error; // Atualiza o erro anterior
    float pid_calculated_value = (this->kp * error) + (this->ki * this->integral) + (this->kd * derivative);

    this->pwmOutput = this->minPwm + pid_calculated_value;

    // Clamp PWM output to the valid range [0, 1023]
    if (this->pwmOutput > 1023.0f) this->pwmOutput = 1023.0f;
    else if (this->pwmOutput < 0) this->pwmOutput = 0; // Ensure PWM is not negative
}

// Inicializa Pinos
void motor_setup() {
  pinMode(EN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_A), a_encoder, FALLING);
  pinMode(EN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_B), b_encoder, FALLING);
  leftWheel.pwm(0, 1);
  rightWheel.pwm(0, 1);
  
}
