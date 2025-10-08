#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include <string>

#define SERVICE_UUID        "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
#define CHARACTERISTIC_UUID "d5593e6b-3328-493a-b3c9-9814683d8e40"

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
        std::string rx = pCharacteristic->getValue();
        String rxValue = String(rx.c_str());
        if (rxValue.length() > 0) {
            receivedData = rxValue;
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
    String device_name = "Neuro_Robot";
    void connect_status();
    void handler();
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

void BluetoothCommunication::handler() {
    connect_status();
    String bleData = receiveData();
    if (bleData.length() > 0) {
        if (bleData.startsWith("CMD:")) {
            String command = bleData.substring(4);
            Serial.println("Comando recebido: " + command);
            processCommand(command);
            // Processar o comando conforme necessário
        }  else if(bleData.startsWith("RQS")) {
            String telemetryData = diffCar.get_telemetry_data();
            sendData(telemetryData.c_str());
        }
    }
}

String BluetoothCommunication::receiveData() {
    String temp = receivedData;
    receivedData = "";
    return temp;
}

void BluetoothCommunication::processCommand(String command) {
    command.trim(); // Remove espaços
    command.toUpperCase(); // Converte para maiúscula
    
    if (command == "START") {
        Serial.println("Comando: Iniciar movimento");
        // Inicia o carrinho W.I.P
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
        // Comando para definir velocidade: VEL:0.50 0.50 (esquerda direita)
        float left_velocity = command.substring(4, command.indexOf(' ', 4)).toFloat();
        float right_velocity = command.substring(command.indexOf(' ', 4) + 1).toFloat();
        Serial.println("Comando: Definir velocidade esquerda " + String(left_velocity) + ", direita " + String(right_velocity));
        diffCar.left_velocity_target = left_velocity;
        diffCar.right_velocity_target = right_velocity;
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