#include "bluetooth.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to end enable it
#endif


#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif


BluetoothSerial SerialBT;


const char *pin = "0000";
String device_name = "esp32";
String state = "steady";

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
            leftWheel.pwm(0, false); // Para o motor esquerdo
            rightWheel.pwm(0, false); // Para o motor direito
            leftWheel.targetRpm = 0; // Reseta a velocidade alvo do motor esquerdo
            rightWheel.targetRpm = 0; // Reseta a velocidade alvo do motor direito
            return 0; // Retorna 0 para indicar que o robô está parado

        } else if (recebido == '1') {
            Serial.println("Andando para frente 100");
            delta_distance = odometry.x_pos_ + 0.15; // Define a distância alvo para 1.5m
            
            Serial.println("Delta Distance: " + String(delta_distance));
            Serial.println("Posição Odometria: (" + String(odometry.x_pos_) + ", " + String(odometry.y_pos_) + ")");
            return 1;
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