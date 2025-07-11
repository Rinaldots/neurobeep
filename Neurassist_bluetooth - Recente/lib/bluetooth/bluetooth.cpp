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
float actual_detection_position = 0;
float last_detection_position = 0;
float actual_detection_time = 0;
float last_detection_time = 0;

bool left_detected = false;
bool right_detected = false;
int last_controll_line = 0; 

// Variáveis globais para o modo rotação
static bool fist_rotate = false;
static bool startup_line = false;
static bool on_controll_line = false; // Variável para verificar se o robô está na linha
static bool counting_distance = false; // Variável para verificar se está contando distância
static unsigned long rotate_last_time = 0;
static float distance_start_position = 0; // Posição onde começou a contar
static float distance_end_position = 0; // Posição onde parou de contar

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

void feedbackBluetooth() {
    // Envia dados de dos sensores via Bluetooth
    readSensors();
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
            SerialBT.println("P-R");
            leftWheel.pwm(0, false); // Para o motor esquerdo
            rightWheel.pwm(0, false); // Para o motor direito
            return 0; 
        } else if (recebido == 'A') {
            SerialBT.println("M-A");
            return 1; // Modo andar para frente com PID lento
        } else if (recebido == 'F') {
            SerialBT.println("M-F");
            return 2; // Modo andar para frente com PID rapido
        } else if (recebido == 'G') {
            return 4;
        } else if (recebido == 'E') {
            feedbackBluetooth();
            return 0;
        } else if (recebido == 'S') {
            SerialBT.println("M-S");
            return 3; // Retorna 3 para indicar que o modo seguir linha está ativ
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
switch (drive_mode)
  {
  case 0: {
        //Modo parar motores
        stopMotors(); // Para os motores
        leftWheel.targetRpm = 0; 
        rightWheel.targetRpm = 0; 
        
        

        break;
    }
  case 1: {
  //Modo andar para frente com PID
    motor_actual_time = millis();
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 100ms 
      setMotorTargetRpm(50, 50);
      motorSpeed(); 
      
      motor_last_time = motor_actual_time;
    }}
    leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup));
    rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup));
    break;
  case 2: {
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 500ms
      setMotorTargetRpm(100, 100); // Define a velocidade alvo dos motores
      motorSpeed();
      motor_last_time = motor_actual_time;
      }
      leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup));
      rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup));
    }
    break;
  case 3: {
    //Modo seguidor de linha  
    if(!startup_line){
      startup_line = true;
      counting_distance = false; 
      Serial.println("Modo seguidor de linha iniciado");
    }
    int motor_actual_time = millis();   
    int speed = 40; // Velocidade padrão para o seguidor de linha
    if (motor_actual_time - motor_last_time > time_update) { 
      motorSpeed();
      motor_last_time = motor_actual_time;
    }
    int pwm_left = static_cast<int>(leftWheel.pwmOutput+motor_startup);    
    int pwm_right = static_cast<int>(rightWheel.pwmOutput+motor_startup); 
    
    actual_detection_position = ((leftWheel.current_position+rightWheel.current_position)/2); 
    
    // Detecta linha de controle (todos os sensores ativos)
    if (CONT_SENSOR_LINE_LEFT && CONT_SENSOR_LINE_RIGHT && CONT_SENSOR_LINE_CENTER) {
      if(!on_controll_line){
          // Primeira detecção da linha de controle
          on_controll_line = true;
          Serial.println("Linha de controle detectada!");
      }
    } else {
      // Não está mais na linha de controle
      if(on_controll_line) {
          // Acabou de sair da linha de controle
          on_controll_line = false;
          if(!counting_distance) {
              counting_distance = true;
              distance_start_position = actual_detection_position;
          } else {
              distance_end_position = actual_detection_position;
              float diferenca = distance_end_position - distance_start_position;            
              if(diferenca < 20 && diferenca > 5) {
                  startup_line = false; 
                  counting_distance = false;
                  return 4;  
              } else {
                  // Distância não adequada, continua e reseta para próxima medição
                  counting_distance = false;
                  last_controll_line = diferenca;
                  SerialBT.println("Linha de central de controle distancia:");
                  SerialBT.print(diferenca);
              }
          }
      }
    } 
    if(CONT_SENSOR_LINE_LEFT && CONT_SENSOR_LINE_RIGHT && CONT_SENSOR_LINE_CENTER) {
      // Está na linha de controle
      left_detected = false; // Reseta a detecção de linha esquerda
      right_detected = false; // Reseta a detecção de linha direita
      
    } else if (CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_RIGHT) {
      on_controll_line = false;
      left_detected = false; // Reseta a detecção de linha esquerda
      right_detected = false; // Reseta a detecção de linha direita
    } else if (CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_RIGHT) {
      on_controll_line = false;
      left_detected = true;
      right_detected = false; 
    } else if (CONT_SENSOR_LINE_RIGHT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT) {
      on_controll_line = false;
      left_detected = false;
      right_detected = true;
    }
    leftWheel.targetRpm = speed;
    rightWheel.targetRpm = speed;
    if (right_detected){
        pwm_right -= 200;
        rightWheel.targetRpm = speed/1.5; // Ajusta a velocidade do motor direito
    }else if (left_detected){
        pwm_left -= 200;
        leftWheel.targetRpm = speed/1.5;// Ajusta a velocidade do motor esquerdo
    }
    
    leftWheel.pwm(pwm_left, false);
    rightWheel.pwm(pwm_right, false);
    
    return 3; // Retorna 3 para indicar que o modo seguidor de linha está ativo
  }
  case 4:{
    // Modo Rotacionar no Fim - Gira até encontrar linha central
    unsigned long rotate_actual_time = millis();   
    int rotate_time = 1500;
    int speed = 80; 

    //SerialBT.println("Modo Rotacionar iniciado");
    if (!fist_rotate){
      fist_rotate = true; // Marca que a rotação foi iniciada
      rotate_last_time = millis(); // Marca o tempo de início da rotação
      SerialBT.println("Iniciando rotação por " + String(rotate_time) + " ms");
    }
    if (motor_actual_time - motor_last_time > time_update) { 
      motorSpeed();
      leftWheel.targetRpm = speed;
      rightWheel.targetRpm = speed;
      motor_last_time = motor_actual_time;
    }
    if(rotate_actual_time - rotate_last_time > rotate_time) {
            
      if (CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_RIGHT) {
        leftWheel.targetRpm = 0; // Para o motor esquerdo
        rightWheel.targetRpm = 0; // Para o motor direito
        leftWheel.pwm(0, false); // Para o motor esquerdo
        rightWheel.pwm(0, false); // Para o motor direito
        SerialBT.println("Linha central detectada após rotação");
        fist_rotate = false;
        rotate_last_time = 0;
        return 3; // Retorna ao modo seguir linha
      } else {
        // Continua girando
        leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup), true);  // Esquerdo para trás
        rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup), false); // Direito para frente
      
      }
    } else {
      // Ainda dentro do tempo de rotação
      leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup), true);  // Esquerdo para trás
      rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup), false); // Direito para frente
    }
    
    return 4;
  }

  
  default:

    break;
  }
  return -1;
}

void readSensors() {
    SerialBT.println();
    SerialBT.println("LS - L: "+String(CONT_SENSOR_LINE_LEFT)+" | C: "+String(CONT_SENSOR_LINE_CENTER)+" | R: "+String(CONT_SENSOR_LINE_RIGHT));
    SerialBT.println("LCL: " + String(last_controll_line) + "| L.enc: " + String(leftWheel.current_position) + "| R.enc: " + String(rightWheel.current_position));
    SerialBT.println();
  }