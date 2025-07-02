#include "motor_controle.h"
#include "odometry.h"
#include "config.h"
#include "sensors.h"
// PID parameters

// Definição das variáveis globais
unsigned long motor_last_time = 0, motor_actual_time = 0;
unsigned long odometry_last_time = 0; // Para controlar o tempo da odometria
float delta_distance = 0.0f; // Variável para armazenar a distância percorrida
MOTOR rightWheel(A_IA, A_IB, false);
MOTOR leftWheel(B_IA, B_IB, false);
PID leftWheel_PID(&leftWheel.currentRpm, &leftWheel.pwmOutput, &leftWheel.targetRpm, 
                  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
PID rightWheel_PID(&rightWheel.currentRpm, &rightWheel.pwmOutput, &rightWheel.targetRpm, 
                   DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
void IRAM_ATTR a_encoder() {
  unsigned long current_time = millis();
  if (current_time - leftWheel.encoder_interrupt_time > debounce_delay) {
    leftWheel.current_position += 1;
    leftWheel.encoder_interrupt_time = current_time;
  }
}
void IRAM_ATTR b_encoder() {
  unsigned long current_time = millis();
  if (current_time - rightWheel.encoder_interrupt_time > debounce_delay) {
    rightWheel.current_position += 1;
    rightWheel.encoder_interrupt_time = current_time;
  }
}

// PIN A -> Input
// PIN B -> PWM Input 
MOTOR::MOTOR(int motor_pin_a, int motor_pin_b, bool is_inverted) {
  this->PIN_A = motor_pin_a;
  this->PIN_PWM_CHANNEL = motor_pin_b;
  this->inverted = is_inverted;
  
  this->current_position = 0;
  this->previous_position = 0;
  this->current_time = 0;
  this->previous_time = 0;

  this->currentRpm = 0.0;
  this->targetRpm = 0.0;
  this->pwmOutput = 0.0;

  ledcAttach(this->PIN_A, freq, resolution);
  ledcAttach(this->PIN_PWM_CHANNEL, freq, resolution);
}


void MOTOR::pwm(int PWM) {
  if ( PWM >= 1023) {
    PWM = 1023; // Limita o PWM a 1023
  } else if (PWM < 0) {
    PWM = 0; // Limita o PWM a -1023
  }
  if (PWM > 0) {
    ledcWrite(PIN_PWM_CHANNEL, PWM);
    ledcWrite(PIN_A, 1023);
  } 
  else {
    ledcWrite(PIN_PWM_CHANNEL, PWM);
    ledcWrite(PIN_A, 0);
  }
}

void MOTOR::calculateCurrentRpm() {
  current_time = millis();
  long current_encoder_pos = this->current_position; 
  unsigned long delta_time_ms = (current_time - previous_time);
  long delta_position = abs(current_encoder_pos - previous_position);
  
  // Proteção contra divisão por zero e valores muito pequenos
  if (delta_time_ms < 10) { // Mínimo 10ms para cálculo válido
      
      return; // Não atualiza se o tempo for muito pequeno
  }
  
  if (delta_position > 0) {
      float calculated_rpm = (static_cast<float>(delta_position) * 60000.0f) / (ENCODER_PULSES_PER_REVOLUTION * delta_time_ms);
      
      float ramp_rate = 10.0f; 
      if (calculated_rpm > this->currentRpm) {
          this->currentRpm += ramp_rate;
          if (this->currentRpm > calculated_rpm) {
          this->currentRpm = calculated_rpm;
          }
      } else {
          this->currentRpm = calculated_rpm;
      }
  } else {
      // Se não houve movimento, decai gradualmente a velocidade
      this->currentRpm *= 0.2f; 
      if (this->currentRpm < 0.1f) {
          this->currentRpm = 0.0f;
      }
  }
  previous_position = current_position;
  previous_time = current_time;
}

// Inicializa os motores e configura os pinos de interrupção para os encoders
void setupMotors() {
  pinMode(EN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_A), a_encoder, FALLING);
  pinMode(EN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_B), b_encoder, FALLING);
  leftWheel.pwm(0);
  rightWheel.pwm(0);
  leftWheel_PID.SetMode(AUTOMATIC);
  rightWheel_PID.SetMode(AUTOMATIC);
  leftWheel_PID.SetSampleTime(200); // Tempo de amostragem em milissegundos (aumentado para mais estabilidade)
  rightWheel_PID.SetSampleTime(200); // Tempo de amostragem em milissegundos
  leftWheel_PID.SetOutputLimits(-motor_startup,(900-motor_startup));  // Limites menos extremos
  rightWheel_PID.SetOutputLimits(-motor_startup, (900-motor_startup)); // Evita PWM muito baixo ou muito alto
}

void stopMotors() {
  leftWheel.targetRpm = 0;
  rightWheel.targetRpm = 0;
  leftWheel.pwm(0);
  rightWheel.pwm(0);
}

// Função que controla a velocidade dos motores usando PID
void motorSpeed(){
  leftWheel.calculateCurrentRpm();
  rightWheel.calculateCurrentRpm();
  
  // Atualiza a odometria com os dados de ambos os motores
  unsigned long current_time = millis();
  if (odometry_last_time > time_update) {
    // Calcula velocidades lineares para odometria (m/s)
    float left_velocity = (leftWheel.currentRpm / 60.0f) * wheel_circumference_;  // RPM para m/s
    float right_velocity = (rightWheel.currentRpm / 60.0f) * wheel_circumference_; // RPM para m/s
    odometry.update(right_velocity, left_velocity); // Note: right first, left second
  }
  odometry_last_time = current_time;
  
  
  leftWheel_PID.Compute();
  rightWheel_PID.Compute();
  
  
  if (leftWheel.targetRpm == 0 && rightWheel.targetRpm == 0) {
    stopMotors();
  }
}

void setMotorTargetRpm(float left_rpm, float right_rpm) {
  leftWheel.targetRpm = left_rpm;
  rightWheel.targetRpm = right_rpm;
}

