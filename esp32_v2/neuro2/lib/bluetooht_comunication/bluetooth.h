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
    bool connected = false;
    String device_name = "ESP32test";
    void connect_status();
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

String BluetoothCommunication::receiveData() {
    if (connected && !receivedData.isEmpty()) {
        String temp = receivedData;
        receivedData = "";
        return temp;
    }
    return "";
}

#endif