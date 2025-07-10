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
int temp_estado = 0;
int temp_estado_motor = 1;
void loop() {
  temp_estado = callbackBluetooth();
  
  if (temp_estado != 9) {
    estado = temp_estado; // Atualiza o estado se houver uma mudança
  }
  
  int temp_estado_motor = motorHandler(estado);
  if (temp_estado_motor != -1) {
    estado = temp_estado_motor;
  }

  //readSensors();
}
