#include "bluetooth.h"
#include "config.h"
#include "motor_controle.h"
#include "odometry.h"
#include "sensors.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to end enable it
#endif


#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif


BluetoothSerial SerialBT;


const char *pin = "0000";
String device_name = "neuro_esp32";
String state = "steady";
float actual_detection = 0;
float last_detection = 0;
bool left_detected = false;
bool right_detected = false;
 

void setupBluetooth() {
  SerialBT.begin(device_name); // Nome do dispositivo Bluetooth
  Serial.printf("O dispositivo com o nome \"%s\" está iniciado.\nAgora você pode emparelhá-lo com o Bluetooth!\n", device_name.c_str());

  #ifdef USE_PIN
  SerialBT.setPin(pin);
  Serial.println("Using PIN");
  #endif
  
  Serial.println("Bluetooth iniciado! Dispositivo disponível para pareamento.");
  Serial.println("Procure pelo dispositivo '" + device_name + "' na lista de dispositivos Bluetooth.");
}



int callbackBluetooth() {
    // Verifica se há um cliente conectado
    static bool clienteConectado = false;
    if (SerialBT.hasClient() && !clienteConectado) {
        Serial.println("Cliente Bluetooth conectado!");
        clienteConectado = true;
    } else if (!SerialBT.hasClient() && clienteConectado) {
        Serial.println("Cliente Bluetooth desconectado!");
        clienteConectado = false;
    }
    
    if (SerialBT.available()) {
        char recebido = SerialBT.read();
        Serial.print("Recebido via Bluetooth: ");
        Serial.println(recebido);

        if (recebido == 'P') {
            Serial.println("Parando o robô");
            leftWheel.pwm(0); // Para o motor esquerdo
            rightWheel.pwm(0); // Para o motor direito
            leftWheel.targetRpm = 0; // Reseta a velocidade alvo do motor esquerdo
            rightWheel.targetRpm = 0; // Reseta a velocidade alvo do motor direito
            return 0; // Retorna 0 para indicar que o robô está parado

        } else if (recebido == '1') {
            SerialBT.println("Modo Seguir Linha Ativado");
            
            return 3;
        } else if (recebido == 'A') {
            Serial.println("Andando para frente");
            setMotorTargetRpm(40, 40); // Define a velocidade alvo dos motores
            return 0; // Retorna 0 para indicar que o robô está parado
        } else if (recebido == 'D') {
            
        } else if (recebido == 'E') {
            
        } else if (recebido == 'S') {
            Serial.println("Modo Seguir Linha Ativado");
            leftWheel.targetRpm = 45; // Reseta a velocidade alvo do motor esquerdo
            rightWheel.targetRpm = 45; // Reseta a velocidade alvo do motor direito
            return 2; // Retorna 2 para indicar que o modo seguir linha está ativ
        }
    }
    return 9; // Retorna 0 se nenhum comando for recebido
}

int motorHandler(int drive_mode) {
 /* 
 drive_mode = 0; // Força o modo de controle por RPM
 drive_mode = 1; // Força o modo de controle por deslocamento
 */
motor_actual_time = millis();
bool on_controll_line = false; // Variável para verificar se o robô está na linha

switch (drive_mode)
  {
  case 0: {
        //Modo controle por RPM
        stopMotors(); // Para os motores
        leftWheel.targetRpm = 0; 
        rightWheel.targetRpm = 0; 
        break;
    }
  case 1: {
  //Modo andar para frente com PID
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 100ms 
      setMotorTargetRpm(140, 140);
      motorSpeed(); 
      leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup));
      rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup));
      motor_last_time = motor_actual_time;
    }}
    break;
  case 2: {
  // Modo andar para frente tentando completar um deslocamento
    bool completed = false;

    if (abs(delta_distance) < odometry.x_pos_) {
      completed = true; // Se delta_distance for negativo, consideramos que o movimento foi completado
      leftWheel.pwm(0);
      rightWheel.pwm(0);
      leftWheel.targetRpm = 0;
      rightWheel.targetRpm = 0;
      return 0; // Não faz nada se delta_distance for zero
    }
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 500ms
      if(!completed) {
        setMotorTargetRpm(100, 100); // Define a velocidade alvo dos motores
        motorSpeed();
        leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup));
        rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup));
        motor_last_time = motor_actual_time;
        Serial.println("Delta Distance: " + String(delta_distance));
        Serial.print(" |  Odometry Position: (" + String(odometry.x_pos_) + ", " + String(odometry.y_pos_) + ")");
      }
    }
    break;
  }
  case 3: {
    //Modo seguidor de linha     
    int speed = 75; // Velocidade padrão para o seguidor de linha
    if (motor_actual_time - motor_last_time > time_update) { 
      motorSpeed();
      motor_last_time = motor_actual_time;
      Serial.println("Left Wheel RPM target: " + String(leftWheel.targetRpm));
      Serial.println("Right Wheel RPM target: " + String(rightWheel.targetRpm));
      Serial.println("leftWheel.velocity: " + String(leftWheel.currentRpm));
      Serial.println("rightWheel.velocity: " + String(rightWheel.currentRpm));
    }
    int pwm_left = static_cast<int>(leftWheel.pwmOutput+motor_startup);    
    int pwm_right = static_cast<int>(rightWheel.pwmOutput+motor_startup); 
    if (CONT_SENSOR_LINE_LEFT && CONT_SENSOR_LINE_RIGHT && CONT_SENSOR_LINE_CENTER) {
      Serial.println("Linha de controle");
      if(!on_controll_line){
          last_detection = actual_detection; // Atualiza a última detecção de linha central
          actual_detection = odometry.x_pos_; // Atualiza a última detecção de linha central
          Serial.print("Ultima detecção: ");
          Serial.print(last_detection);
          Serial.print(" | Detecção atual: ");
          Serial.print(actual_detection);
          Serial.print(" | Diferença: ");
          Serial.println(actual_detection - last_detection);
      }

      on_controll_line = true;
      
      if((actual_detection - last_detection < 0.8) && odometry.x_pos_ > 0.8) {
        Serial.print("Stop detecatado");
        stopMotors(); // Para os motores
        leftWheel.pwm(0); // Para o motor esquerdo
        rightWheel.pwm(0); // Para o motor direito
        Serial.println("Parando o robô");
        
      } else if(actual_detection - last_detection > 1.5) {
        Serial.println("Entre linhas detectado!");
        Serial.print("Ultima detecção: ");
        Serial.print(last_detection);
        Serial.print(" | Detecção atual: ");
        Serial.print(actual_detection);
        Serial.print(" | Diferença: ");
        Serial.println(actual_detection - last_detection);
      }
    } else if (CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_RIGHT) {

      on_controll_line = false;
      left_detected = false; // Reseta a detecção de linha esquerda
      right_detected = false; // Reseta a detecção de linha direita
      // Mantém velocidade normal
    } else if (CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_RIGHT) {
      on_controll_line = false;
      left_detected = true;
      right_detected = false; 
    } else if (CONT_SENSOR_LINE_RIGHT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT) {
      on_controll_line = false;
      left_detected = false;
      right_detected = true;
    } else if(!CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_RIGHT) {
    }

    if (right_detected){
        Serial.println("Indo para esquerda");
        SerialBT.println("Indo para esquerda");
        pwm_right -= 250;
    }else if (left_detected){
        pwm_left -= 250;
        Serial.println("Indo para direita");
        SerialBT.println("Indo para direita");
    } else {
        Serial.println("Linha central detectada, mantendo velocidade");
        SerialBT.println("Linha central detectada, mantendo velocidade");
    }

    leftWheel.pwm(pwm_left);
    rightWheel.pwm(pwm_right);
    leftWheel.targetRpm = speed;
    rightWheel.targetRpm = speed;

    
    return 3; // Retorna 3 para indicar que o modo seguidor de linha está ativo
    
  }
  

  
  default:

    break;
  }
  return -1;
}