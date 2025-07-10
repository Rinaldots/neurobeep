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
int last_controll_line = 0; // Variável para armazenar a última linha de controle detectada

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
    // Envia dados de odometria e sensores via Bluetooth
    SerialBT.print("Odometry Position: (");
    SerialBT.print(odometry.x_pos_);
    SerialBT.print(", ");
    SerialBT.print(odometry.y_pos_);
    SerialBT.print(") Heading: ");
    SerialBT.println(odometry.heading_);

    SerialBT.print("Line Sensors - Left: ");
    SerialBT.print(CONT_SENSOR_LINE_LEFT);
    SerialBT.print(" | Center: ");
    SerialBT.print(CONT_SENSOR_LINE_CENTER);
    SerialBT.print(" | Right: ");
    SerialBT.println(CONT_SENSOR_LINE_RIGHT);
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
            return 2;
        } else if (recebido == 'D') {
            
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
        
        //SerialBT.println("Diferença: "+String(last_controll_line));

        break;
    }
  case 1: {
  //Modo andar para frente com PID
    motor_actual_time = millis();
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 100ms 
      setMotorTargetRpm(60, 60);
      motorSpeed(); 
      leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup));
      rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup));
      motor_last_time = motor_actual_time;
    }}
    break;
  case 2: {
  // Modo andar para frente tentando completar um deslocamento
    
    if (motor_actual_time - motor_last_time > time_update) { // Atualiza a cada 500ms
      setMotorTargetRpm(60, 60); // Define a velocidade alvo dos motores
      motorSpeed();
      motor_last_time = motor_actual_time;
    }
    leftWheel.pwm(static_cast<int>(leftWheel.pwmOutput+motor_startup));
    rightWheel.pwm(static_cast<int>(rightWheel.pwmOutput+motor_startup));
    break;
  }
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
    
    if (CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_RIGHT) {
      left_detected = false; // Reseta a detecção de linha esquerda
      right_detected = false; // Reseta a detecção de linha direita
      
    } else if (CONT_SENSOR_LINE_LEFT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_RIGHT) {
      left_detected = true;
      right_detected = false; 
    } else if (CONT_SENSOR_LINE_RIGHT && !CONT_SENSOR_LINE_CENTER && !CONT_SENSOR_LINE_LEFT) {
      left_detected = false;
      right_detected = true;
    }
    leftWheel.targetRpm = speed;
    rightWheel.targetRpm = speed;
    if (right_detected){
        pwm_right -= 250;
        //rightWheel.targetRpm = speed/1.5; // Ajusta a velocidade do motor direito
    }else if (left_detected){
        pwm_left -= 250;
        //leftWheel.targetRpm = speed/1.5;// Ajusta a velocidade do motor esquerdo
    } else {
    }

    SerialBT.println(); 
    SerialBT.print("Detecção de linha: ");
    if (CONT_SENSOR_LINE_LEFT) {
      SerialBT.print("Esquerda ");
    }
    if (CONT_SENSOR_LINE_CENTER) {
      SerialBT.print("Centro ");
    }
    if (CONT_SENSOR_LINE_RIGHT) {
      SerialBT.print("Direita ");
    }
    SerialBT.println(); 
    SerialBT.print("Contador esquerdo: ");
    SerialBT.print(leftWheel.current_position);
    SerialBT.print(" | Contador direito: ");
    SerialBT.println(rightWheel.current_position);

    leftWheel.pwm(pwm_left, false);
    rightWheel.pwm(pwm_right, false);
    

    return 3; // Retorna 3 para indicar que o modo seguidor de linha está ativo
  }
  default:

    break;
  }
  return -1;
}