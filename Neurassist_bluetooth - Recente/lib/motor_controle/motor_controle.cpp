
#include "motor_controle.h"
#include "odometry.h"
#include "config.h"
// PID parameters

// Definição das variáveis globais
unsigned long motor_last_time = 0, motor_actual_time = 0;
float delta_distance = 0.0f; // Variável para armazenar a distância percorrida
MOTOR rightWheel(B_IA, B_IB, false, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN_PWM);
MOTOR leftWheel(A_IB, A_IA, false, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN_PWM);
PID leftWheel_PID(&leftWheel.currentRpm, &leftWheel.pwmOutput, &leftWheel.targetRpm, 
                  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
PID rightWheel_PID(&rightWheel.currentRpm, &rightWheel.pwmOutput, &rightWheel.targetRpm, 
                   DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
void IRAM_ATTR a_encoder() {
  unsigned long current_time = millis();
  if (current_time - leftWheel.encoder_interrupt_time > debounce_delay) {
    leftWheel.current_position++;
    leftWheel.encoder_interrupt_time = current_time;
  }
}
void IRAM_ATTR b_encoder() {
  unsigned long current_time = millis();
  if (current_time - rightWheel.encoder_interrupt_time > debounce_delay) {
    rightWheel.current_position++;
    rightWheel.encoder_interrupt_time = current_time;
  }
}

MOTOR::MOTOR(int motor_pin_a, int motor_pin_b, bool is_inverted) {
  this->PIN_A = motor_pin_a;
  this->PIN_B = motor_pin_b;
  this->inverted = is_inverted;
  
  this->current_position = 0;
  this->previous_position = 0;
  this->current_time = 0;
  this->previous_time = 0;

  this->currentRpm = 0.0;
  this->targetRpm = 0.0;
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
  current_time = millis();
  long current_encoder_pos = this->current_position; 
  unsigned long delta_time_ms = (current_time - previous_time);

  
    long delta_position = abs(current_encoder_pos - previous_position);

    if (delta_time_ms > 0) { // Evita divisão por zero
        this->currentRpm = (static_cast<float>(delta_position) * 60000.0f) / (ENCODER_PULSES_PER_REVOLUTION * delta_time_ms);
    } else {
        this->currentRpm = 0; 
    }
    previous_position = current_position;
    previous_time = current_time;
    // Atualiza a odometria
    float avg_rps_x = (leftWheel.currentRpm + rightWheel.currentRpm) * 0.5f / 20.0f;
    float linear_x = avg_rps_x * wheel_circumference_;
    float avg_rps_a = (-leftWheel.currentRpm + rightWheel.currentRpm) * 0.5f / 20.0f;
    float angular_z = (avg_rps_a * wheel_circumference_) / (wheels_y_distance_ * 0.5f);
    float vel_dt = (current_time - previous_time) * 0.001f;
    odometry.update(vel_dt, linear_x, 0, angular_z);
  

  
  
}


// Inicializa os motores e configura os pinos de interrupção para os encoders
void setupMotors() {
  pinMode(EN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_A), a_encoder, FALLING);
  pinMode(EN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_B), b_encoder, FALLING);
  leftWheel.pwm(0, 1);
  rightWheel.pwm(0, 1);
  leftWheel_PID.SetMode(AUTOMATIC);
  rightWheel_PID.SetMode(AUTOMATIC);
  leftWheel_PID.SetOutputLimits(0, 1023);
  rightWheel_PID.SetOutputLimits(0, 1023);
  
}

// Função que é chamada periodicamente para atualizar o pwm dos motores
void motorHandler(int drive_mode) {
 // drive_mode = 0; // Força o modo de controle por RPM
 // drive_mode = 1; // Força o modo de controle por deslocamento
 motor_actual_time = millis();
switch (drive_mode)
  {
  case 0:
    
    
  if (motor_actual_time - motor_last_time > 500) {
    leftWheel.calculateCurrentRpm();
    rightWheel.calculateCurrentRpm();
    float left_gap = abs(leftWheel.currentRpm - leftWheel.targetRpm);
    float right_gap = abs(rightWheel.currentRpm - rightWheel.targetRpm);
    
    if (left_gap > 3){
      Serial.println("Left gap: " + String(left_gap));
      if (left_gap < 20)
        {  //we're close to setpoint, use conservative tuning parameters
          leftWheel_PID.SetTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
        }
      else
        {
          //we're far from setpoint, use aggressive tuning parameters
          leftWheel_PID.SetTunings(AGG_KP, AGG_KI, AGG_KD);
        }
        leftWheel_PID.Compute();
    }
    if (right_gap > 3 || !leftWheel.currentRpm == 0){
      Serial.println("Right gap: " + String(right_gap));
      if (right_gap < 20)
        {  //we're close to setpoint, use conservative tuning parameters
          rightWheel.pwm(static_cast<int>(rightWheel.minPwm), true);
          rightWheel_PID.SetTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
        }
      else
        {
          //we're far from setpoint, use aggressive tuning parameters
          rightWheel_PID.SetTunings(AGG_KP, AGG_KI, AGG_KD);
        }
      
        rightWheel_PID.Compute();
    }

    

    leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput), true);
    rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput), true);
    motor_last_time = motor_actual_time;

    Serial.print("PWM Left: ");
    Serial.print(leftWheel.pwmOutput);
    Serial.print(" | PWM Right: ");
    Serial.print(rightWheel.pwmOutput);
    Serial.print("Velocity Left: ");
    Serial.print(leftWheel.currentRpm);
    Serial.print(" | Right: ");
    Serial.println(rightWheel.currentRpm);
  }
    break;
  case 1:
    // Implementar o controle por deslocamento aqui
    if(abs(delta_distance) < 0) {
      if (motor_last_time - motor_actual_time > 200) {
      leftWheel.calculateCurrentRpm();
      rightWheel.calculateCurrentRpm();

      leftWheel.calculatePid();
      rightWheel.calculatePid();

      leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput), true);
      rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput), true);
      motor_last_time = motor_actual_time;
    }
    Serial.println("Delta Distance: " + String(delta_distance));
    Serial.println("Odometry Position: (" + String(odometry.x_pos_) + ", " + String(odometry.y_pos_) + ")");
    }
    break;
  default:

    break;
  }
}