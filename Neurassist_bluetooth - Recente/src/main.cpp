#include <Arduino.h>
#include <BluetoothSerial.h>
#include "config.h"  // Inclui as configurações do projeto
#include "motor_controle.h"


bool logica = false;
int pulseCount_right = 0; // Contador de pulsos
int CONT_SENSOR_LINE_LEFT = 0; // Contador para o sensor de linha esquerdo
int CONT_SENSOR_LINE_RIGHT = 0; // Contador para o sensor de linha direito
int CONT_SENSOR_LINE = 0;
unsigned long lastUpdateTime = 0; // Tempo da última atualização

const char *pin = "0000";
String device_name = "esp32";
String state = "steady";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to end enable it
#endif


#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif


BluetoothSerial SerialBT;


// Funções de interrupção para sensores de linha
void IRAM_ATTR handleLineLeft() {
  CONT_SENSOR_LINE_LEFT++; // Incrementa o contador do sensor de linha esquerdo
}

void IRAM_ATTR handleLine() {
  CONT_SENSOR_LINE++; // Incrementa o contador do sensor de linha esquerdo
}

void IRAM_ATTR handleLineRight() {
  CONT_SENSOR_LINE_RIGHT++; // Incrementa o contador do sensor de linha direito
}

void comando(String cmd) {
  int vel = 1024;
  if (cmd == "parar") {
    Serial.println("Parando o robô");
    leftWheel.pwm(0, false); // Para o motor esquerdo
    rightWheel.pwm(0, false); // Para o motor direito
  } else if (cmd == "andar") {
    Serial.println("Andando para frente");
    leftWheel.pwm(vel, true); // Define a velocidade do motor esquerdo
    rightWheel.pwm(vel, true); // Define a velocidade do motor direito
  } else if (cmd == "tras") {
    Serial.println("Andando para trás");
    leftWheel.pwm(vel, false); // Define a velocidade do motor esquerdo para trás
    rightWheel.pwm(vel, false); // Define a velocidade do motor direito para trás
  } else if (cmd == "direita") {
    Serial.println("Virando para a direita");
    leftWheel.pwm(vel, true); // Motor esquerdo avança
    rightWheel.pwm(vel / 2, false); // Motor direito reduz a velocidade
  } else if (cmd == "esquerda") {
    Serial.println("Virando para a esquerda");
    leftWheel.pwm(vel / 2, false);  
    rightWheel.pwm(vel, true); // Motor direito avança
    
  }
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(9600);
  SerialBT.begin(device_name); // Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  #ifdef USE_PIN
  SerialBT.setPin(pin);
  Serial.println("Using PIN");
  #endif
  //Serial.println("Iniciando contagem de pulsos com módulo IR e sensores de linha");

  // Configura os pinos e a conexão Wi-Fi
  pinMode(IR_SENSOR_PIN_left, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN_right, INPUT_PULLUP);
  pinMode(SENSOR_LINE_LEFT, INPUT_PULLUP); // Configura o pino do sensor de linha esquerdo
  pinMode(SENSOR_LINE_RIGHT, INPUT_PULLUP); // Configura o pino do sensor de linha direito 
  // Configura interrupções para sensores de linha
  attachInterrupt(digitalPinToInterrupt(SENSOR_LINE_LEFT), handleLineLeft, FALLING);  // A linha é detectada quando o sinal é baixo
  attachInterrupt(digitalPinToInterrupt(SENSOR_LINE_RIGHT), handleLineRight, FALLING);  // A linha é detectada quando o sinal é baixo
  attachInterrupt(digitalPinToInterrupt(SENSOR_LINE), handleLine, FALLING);
  lastUpdateTime = millis();
}

void loop() {
  int estadoSensorEsquerdo = digitalRead(SENSOR_LINE_LEFT);
  int estadoSensorDireito = digitalRead(SENSOR_LINE_RIGHT);
  //leftWheel.pwm(1024, true); // Define a velocidade do motor esquerdo
  //rightWheel.pwm(1024, true); // Define a velocidade do motor direito
  
  logica = true;
  if (logica == true){
    // Lógica de seguidor de linha com controle dos pinos de direção
    if (estadoSensorEsquerdo == LOW && estadoSensorDireito == LOW) {
      
    } else if (estadoSensorEsquerdo == HIGH && estadoSensorDireito == LOW) {
      
  
      Serial.println("Virando para a direita");
    } else if (estadoSensorEsquerdo == LOW && estadoSensorDireito == HIGH) {
      
  
      Serial.println("Seguindo em frente");
    } else {
  
      Serial.println("Seguindo em frente");
    }
  }
  if (SerialBT.available()) {
    char recebido = SerialBT.read();
    Serial.print("Recebido via Bluetooth: ");
    Serial.println(recebido);

    
    if (recebido == 'P') {
      comando("parar");
      logica = false;
    }
    else if (recebido == 'A') {
      comando("andar");
      logica = false;
    }
    else if (recebido == 'T') {
      comando("tras");
      logica = false;
    }
    else if (recebido == 'D') {
      comando("direita");
      logica = false;
    }
    else if (recebido == 'E') {
      comando("esquerda");
      logica = false;
    }
    else if (recebido == 'S') {
      logica = true;
    }
  }

  //}
  
}
