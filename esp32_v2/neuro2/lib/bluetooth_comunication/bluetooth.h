#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Variáveis globais para callback
bool deviceConnected = false;
String receivedData = "";

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Cliente BLE conectado");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Cliente BLE desconectado");
        BLEDevice::startAdvertising(); // Restart advertising
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            receivedData = rxValue;
            Serial.println("Dados recebidos via BLE: " + receivedData);
        }
    }
};

class BluetoothCommunication { 
private:
    BLEServer* pServer;
    BLECharacteristic* pCharacteristic;
    BLEService* pService;
    
public:
    void begin();
    void sendData(const char* data);
    String receiveData();
    void processCommand(String command);
    bool connected = false;
    String device_name = "ESP32test";
    void connect_status();
    void handler(String data);
};

void BluetoothCommunication::begin() {
    BLEDevice::init(device_name.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    
    Serial.println("O dispositivo BLE esta pronto para conexao");
}

void BluetoothCommunication::connect_status() {
    connected = deviceConnected;
}

void BluetoothCommunication::sendData(const char* data) {
    if (connected && pCharacteristic) {
        pCharacteristic->setValue(data);
        pCharacteristic->notify();
    }
}

void BluetoothCommunication::handler(String data) {
    if (data.startsWith("CMD:")) {
        String command = data.substring(4);
        Serial.println("Comando recebido: " + command);
        processCommand(command);
        // Processar o comando conforme necessário
    }  else if(data.startsWith("RQS:")) {
        String telemetryData = diffCar.get_telemetry_data();
        bluetooth.sendData(telemetryData.c_str());
    }
}

String BluetoothCommunication::receiveData() {
    if (connected && !receivedData.isEmpty()) {
        String temp = receivedData;
        receivedData = "";
        return temp;
    }
    return "";
}

void BluetoothCommunication::processCommand(String command) {
    command.trim(); // Remove espaços
    command.toUpperCase(); // Converte para maiúscula
    
    if (command == "START") {
        Serial.println("Comando: Iniciar movimento");
        // Inicia o carrinho W.I.P
    }
    else if (command == "PARAR") {
        Serial.println("Comando: Parar movimento");
        diffCar.left_velocity_target = 0.0;
        diffCar.right_velocity_target = 0.0;
    }
    else if (command == "FRENTE") {
        Serial.println("Comando: Mover para frente");
        diffCar.left_motor_dir = 1;
        diffCar.right_motor_dir = 1;
        diffCar.left_velocity_target = 0.2;
        diffCar.right_velocity_target = 0.2;
    }
    else if (command == "TRAS") {
        Serial.println("Comando: Mover para trás");
        diffCar.left_motor_dir = -1;
        diffCar.right_motor_dir = -1;
        diffCar.left_velocity_target = 0.2;
        diffCar.right_velocity_target = 0.2;
    }
    else if (command == "ESQUERDA") {
        Serial.println("Comando: Virar à esquerda");
        diffCar.left_velocity_target = 0.1;
        diffCar.right_velocity_target = 0.2;
    }
    else if (command == "DIREITA") {
        Serial.println("Comando: Virar à direita");
        diffCar.left_velocity_target = 0.2;
        diffCar.right_velocity_target = 0.1;
    }
    else if (command == "RESET_KALMAN") {
        Serial.println("Comando: Reset Filtro de Kalman");
        diffCar.reset_kalman_filter();
    }
    else if (command == "CALIBRATE_IMU") {
        Serial.println("Comando: Calibrar IMU");
        diffCar.calibrate_imu();
    }
    else if (command.startsWith("VEL:")) {
        // Comando para definir velocidade: VEL:0.5
        float velocity = command.substring(4).toFloat();
        Serial.println("Comando: Definir velocidade " + String(velocity));
        if (velocity < 0.0) {
            diffCar.left_motor_dir = -1;
            diffCar.right_motor_dir = -1;
        } else {
            diffCar.left_motor_dir = 1;
            diffCar.right_motor_dir = 1;
        }
        diffCar.left_velocity_target = velocity;
        diffCar.right_velocity_target = velocity;
    }
    else if (command.startsWith("TURN:")) {
        // Comando para virar: TURN:-0.5 (negativo = esquerda, positivo = direita)
        float turn_rate = command.substring(5).toFloat();
        Serial.println("Comando: Virar " + String(turn_rate));
        diffCar.left_velocity_target = 0.2 - turn_rate;
        diffCar.right_velocity_target = 0.2 + turn_rate;
    }
    else {
        Serial.println("Comando desconhecido: " + command);
        sendData("ERROR: Comando não reconhecido");
    }
}


#endif