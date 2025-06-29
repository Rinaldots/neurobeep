#include <Arduino.h>
#include "config.h"  // Inclui as configurações do projeto
#include "motor_controle.h"
#include "sensors.h" // Inclui as funções de controle dos sensores
#include "bluetooth.h" // Inclui as funções de controle do Bluetooth
#include "odometry.h" // Inclui as funções de odometria

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);
  setupBluetooth(); // Configura o Bluetooth
  setupMotors(); // Configura os motores
  setupSensors(); // Configura os sensores
}
int estado = 0;
void loop() {

  estado = callbackBluetooth();
  
  motorHandler(estado);
}
/*
int estadoSensorEsquerdo = digitalRead(SENSOR_LINE_LEFT);
  int estadoSensorDireito = digitalRead(SENSOR_LINE_RIGHT);

  if (true){
    // Lógica de seguidor de linha com controle dos pinos de direção
    if (estadoSensorEsquerdo == LOW && estadoSensorDireito == LOW) {
      
    } else if (estadoSensorEsquerdo == HIGH && estadoSensorDireito == LOW) {
      
  
      Serial.println("Virando para a direita");
    } else if (estadoSensorEsquerdo == LOW && estadoSensorDireito == HIGH) {
      
  
      Serial.println("Seguindo em frente");
    } else {
  
      Serial.println("Seguindo em frente");
    }


*/