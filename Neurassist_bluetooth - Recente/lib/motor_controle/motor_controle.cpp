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

// PIN A -> Input
// PIN B -> PWM Input 
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
  if (PWM > 0) {
    ledcWrite(PIN_B, 1023-PWM);
    ledcWrite(PIN_A, 1023);
  } 
  else {
    ledcWrite(PIN_B, 0);
    ledcWrite(PIN_A, 0);
  }

  
}

void MOTOR::calculateCurrentRpm() {
  current_time = millis();
  long current_encoder_pos = this->current_position; 
  unsigned long delta_time_ms = (current_time - previous_time);
  long delta_position = abs(current_encoder_pos - previous_position);
  
  if (delta_position > 0) {
      this->currentRpm = (static_cast<float>(delta_position) * 60000.0f) / (ENCODER_PULSES_PER_REVOLUTION * delta_time_ms);
  } else {
      // Se não houve movimento, decai gradualmente a velocidade
      this->currentRpm *= 0.01f; // Decaimento gradual
      if (this->currentRpm < 1.0f) {
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
  leftWheel.pwm(0, 1);
  rightWheel.pwm(0, 1);
  leftWheel_PID.SetMode(AUTOMATIC);
  rightWheel_PID.SetMode(AUTOMATIC);
  leftWheel_PID.SetSampleTime(250); // Tempo de amostragem em milissegundos (reduzido para melhor resposta)
  rightWheel_PID.SetSampleTime(250); // Tempo de amostragem em milissegundos
  leftWheel_PID.SetOutputLimits(0, 1023);
  rightWheel_PID.SetOutputLimits(0, 1023);
}

void stopMotors() {
  leftWheel.targetRpm = 0;
  rightWheel.targetRpm = 0;
  leftWheel.pwm(0, true);
  rightWheel.pwm(0, true);
}

// Função que controla a velocidade dos motores usando PID
void motorSpeed(){
  leftWheel.calculateCurrentRpm();
  rightWheel.calculateCurrentRpm();
  
  // Atualiza a odometria com os dados de ambos os motores
  unsigned long current_time = millis();
  if (odometry_last_time > 0) { // Só atualiza se não for a primeira execução
    // Calcula velocidades lineares para odometria (m/s)
    float left_velocity = (leftWheel.currentRpm / 60.0f) * wheel_circumference_;  // RPM para m/s
    float right_velocity = (rightWheel.currentRpm / 60.0f) * wheel_circumference_; // RPM para m/s
    odometry.update(right_velocity, left_velocity); // Note: right first, left second
  }
  odometry_last_time = current_time;
  
  float left_gap = abs(leftWheel.currentRpm - leftWheel.targetRpm);
  float right_gap = abs(rightWheel.currentRpm - rightWheel.targetRpm);
  if (left_gap > 2){
    if (left_gap < 10){  //we're close to setpoint, use conservative tuning parameters
        leftWheel_PID.SetTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);}
    else{//we're far from setpoint, use aggressive tuning parameters
        leftWheel_PID.SetTunings(AGG_KP, AGG_KI, AGG_KD);}
      leftWheel_PID.Compute();
      leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput), true);
  }
  if (right_gap > 2){
    if (right_gap < 10){  //we're close to setpoint, use conservative tuning parameters
        rightWheel_PID.SetTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);}
    else{//we're far from setpoint, use aggressive tuning parameters
        rightWheel_PID.SetTunings(AGG_KP, AGG_KI, AGG_KD);}
      rightWheel_PID.Compute();
      rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput), true);
  }
  if (leftWheel.targetRpm == 0 && rightWheel.targetRpm == 0) {
    stopMotors();
  }
}

// Debug function to monitor encoder activity
void debugEncoders() {
  static unsigned long last_debug_time = 0;
  unsigned long current_time = millis();
  
  if (current_time - last_debug_time > 2000) { // Debug a cada 2 segundos
    Serial.println("=== ENCODER DEBUG ===");
    Serial.print("Left Encoder Position: ");
    Serial.print(leftWheel.current_position);
    Serial.print(" | Right Encoder Position: ");
    Serial.println(rightWheel.current_position);
    Serial.print("Left Previous Position: ");
    Serial.print(leftWheel.previous_position);
    Serial.print(" | Right Previous Position: ");
    Serial.println(rightWheel.previous_position);
    Serial.print("Left Target RPM: ");
    Serial.print(leftWheel.targetRpm);
    Serial.print(" | Right Target RPM: ");
    Serial.println(rightWheel.targetRpm);
    Serial.println("==================");
    last_debug_time = current_time;
  }
}

// Function to set target RPM for both motors
void setMotorTargetRpm(float left_rpm, float right_rpm) {
  leftWheel.targetRpm = left_rpm;
  rightWheel.targetRpm = right_rpm;
}

// Motor calibration function to identify differences between motors


void motorHandler(int drive_mode) {
 /* 
 drive_mode = 0; // Força o modo de controle por RPM
 drive_mode = 1; // Força o modo de controle por deslocamento
 */
motor_actual_time = millis();
int time_update = 250;
switch (drive_mode)
  {
  case 0: 
  //Modo andar para frente com PID
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 100ms 
      setMotorTargetRpm(140, 140);
      motorSpeed();
      debugEncoders(); 
      
      
      motor_last_time = motor_actual_time;
      
      Serial.print("PWM Left: ");
      Serial.print(leftWheel.pwmOutput);
      Serial.print(" | PWM Right: ");
      Serial.print(rightWheel.pwmOutput);
      Serial.print(" | Velocity Left: ");
      Serial.print(leftWheel.currentRpm);
      Serial.print(" | Right: ");
      Serial.print(rightWheel.currentRpm);
      Serial.print("Target RPM Left: ");
      Serial.print(leftWheel.targetRpm);
      Serial.print(" | Target RPM Right: ");
      Serial.println(rightWheel.targetRpm);
    }
    break;
  case 1: {
  // Modo andar para frente tentando completar um deslocamento
    bool completed = false;

    if (abs(delta_distance) < odometry.x_pos_) {
      completed = true; // Se delta_distance for negativo, consideramos que o movimento foi completado
      //Serial.println("Delta Distance is zero, waiting for command...");
      leftWheel.pwm(0, true);
      rightWheel.pwm(0, true);
      leftWheel.targetRpm = 0;
      rightWheel.targetRpm = 0;
      return; // Não faz nada se delta_distance for zero
    }
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 500ms
      if(!completed) {
        
        setMotorTargetRpm(45, 45); // Define a velocidade alvo dos motores
        motorSpeed();

        motor_last_time = motor_actual_time;
        Serial.println("Delta Distance: " + String(delta_distance));
        Serial.print(" |  Odometry Position: (" + String(odometry.x_pos_) + ", " + String(odometry.y_pos_) + ")");
      }
    }
    break;
  }
  case 2: {
    //Modo seguidor de linha     
    setMotorTargetRpm(145, 145);
    if (motor_actual_time - motor_last_time > time_update) { 
      if (CONT_SENSOR_LINE_LEFT && CONT_SENSOR_LINE_RIGHT && CONT_SENSOR_LINE_CENTER) {
        Serial.println("Linha de controle");
      } else if (CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_RIGHT) {
        //Serial.println("Linha central detectada!");
        
      } else if (CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_RIGHT) {
        Serial.println("Linha esquerda");
        leftWheel.targetRpm = 30; // Ajusta a velocidade do motor esquerdo
        rightWheel.targetRpm = 60; // Ajusta a velocidade do motor direito
      } else if (CONT_SENSOR_LINE_RIGHT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT) {
        Serial.println("Linha direita");
        leftWheel.targetRpm = 60; // Ajusta a velocidade do motor esquerdo
        rightWheel.targetRpm = 30; // Ajusta a velocidade do motor direito
      } else if(!CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_RIGHT) {
        //Serial.println("Nenhuma linha detectada, girando para esquerda");
        leftWheel.pwm(1024,true); // Para os motores se nenhuma linha for detect
        rightWheel.pwm(1024,false); // Para os motores se nenhuma linha for detectada
      }
      motorSpeed();
      motor_last_time = motor_actual_time;
      Serial.print("PWM Left: ");
      Serial.print(leftWheel.pwmOutput);
      Serial.print(" | PWM Right: ");
      Serial.print(rightWheel.pwmOutput);
      Serial.print(" | Velocity Left: ");
      Serial.print(leftWheel.currentRpm);
      Serial.print(" | Right: ");
      Serial.print(rightWheel.currentRpm);
      Serial.print("Target RPM Left: ");
      Serial.print(leftWheel.targetRpm);
      Serial.print(" | Target RPM Right: ");
      Serial.println(rightWheel.targetRpm);
    }

    }
  

  
  default:

    break;
  }
}
