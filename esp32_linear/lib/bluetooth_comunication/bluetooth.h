#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

#define SERVICE_UUID "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
#define CHARACTERISTIC_UUID "d5593e6b-3328-493a-b3c9-9814683d8e40"

// Compile-time debug for BLE; set to 1 to enable verbose Serial prints
#ifndef DEBUG_BLE
#define DEBUG_BLE 1
#endif

struct __attribute__((packed)) TelemetryData {
    uint16_t header;                // 0  - 2
    int32_t encoder_left_count;     // 2  - 4
    int32_t encoder_right_count;    // 6  - 4
    float left_velocity_ms;         // 10 - 4
    float right_velocity_ms;        // 14 - 4
    float left_velocity_target;     // 18 - 4
    float right_velocity_target;    // 22 - 4
    float left_motor_pwm;           // 26 - 4
    float right_motor_pwm;          // 30 - 4
    float left_gain;                // 34 - 4
    float right_gain;               // 38 - 4
    float odom_x;                   // 42 - 4
    float odom_y;                   // 46 - 4
    float odom_theta;               // 50 - 4
    float odom_vx;                  // 54 - 4
    float odom_vy;                  // 58 - 4
    float odom_omega;               // 62 - 4
    float accel_x;                  // 66 - 4
    float accel_y;                  // 70 - 4
    float accel_z;                  // 74 - 4
    int16_t line_distance_mm;       // 78 - 2
    uint16_t line_marker_count;     // 80 - 2
    float line_marker_distance_m;   // 82 - 4
    float gps_latitude;             // 86 - 4
    float gps_longitude;            // 90 - 4
    float gps_altitude;             // 94 - 4
    float gps_speed;                // 98 - 4
    uint8_t gps_valid;              // 102 - 1
    char rfid_uid[12];              // 103 - 12
};

// Variáveis globais para callback
bool deviceConnected = false;
String receivedData = "";

// Telemetry cache / limits
#ifndef TELEMETRY_MAX_LEN
#define TELEMETRY_MAX_LEN 130
#endif

// Per-part length used when splitting telemetry into multiple packets
#ifndef TELEMETRY_PART_LEN
#define TELEMETRY_PART_LEN 130
#endif

BLECharacteristic *pCharacteristic;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        if(DEBUG_BLE) Serial.println(">> DISPOSITIVO CONECTADO <<");
    };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        if(DEBUG_BLE) Serial.println(">> DISPOSITIVO DESCONECTADO <<");
        // Reinicia advertising para conectar de novo
        BLEDevice::startAdvertising(); 
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        if(DEBUG_BLE) Serial.println(">> DISPOSITIVO CONECTADO <<");
    };
    
    // Função para monitorar inclui latência/intervalo
    void onMtuChange(uint16_t MTU, uint16_t conn_id) {
        if(DEBUG_BLE) Serial.printf("MTU alterado para: %d\n", MTU);
    }

    void onWrite(BLECharacteristic *pCharacteristic)
    {
        auto tmp = pCharacteristic->getValue();
        String rxValue = String(tmp.c_str());
        if (rxValue.length() > 0)
        {
            receivedData = rxValue;
        }
    }
};

class BluetoothCommunication
{
private:
    BLEServer *pServer;
    BLECharacteristic *pCharacteristic;
    BLEService *pService;
    uint8_t telemetry_cache_binary[120];
    size_t telemetry_cache_binary_len;

public:
    void begin();
    void sendData(const char *data);
    void updateTelemetryCache();
    String receiveData();
    void processCommand(String command);
    bool connected = false;
    String device_name = "Neuro_Robot";
    void connect_status();
    void handler();

    TelemetryData telemetry_data;
};

void BluetoothCommunication::begin()
{
    BLEDevice::init(device_name.c_str());
    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE |
        BLECharacteristic::PROPERTY_WRITE |    // Permite Escrita
        BLECharacteristic::PROPERTY_WRITE_NR   // Permite Escrita Sem Resposta
    );

    // 2. Característica de ESCRITA (Comandos do Celular) - UUID c1d5...
    // ADICIONADO: PROPERTY_WRITE_NR para permitir envio rápido sem travar o app
    
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCallbacks());


    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();


    if (DEBUG_BLE)
        Serial.println("O dispositivo BLE esta pronto para conexao");
}

void BluetoothCommunication::connect_status()
{
    connected = deviceConnected;
}

void BluetoothCommunication::sendData(const char *data)
{
    if (!(connected && pCharacteristic))
        return;

    // send up to TELEMETRY_MAX_LEN bytes to avoid large packet fragmentation
    size_t len = strlen(data);
    if (len > TELEMETRY_PART_LEN)
        len = TELEMETRY_PART_LEN;

    // Use explicit setValue with length to avoid extra allocations
    pCharacteristic->setValue((uint8_t *)data, (size_t)len);
    pCharacteristic->notify();
}

void BluetoothCommunication::updateTelemetryCache()
{
    
}

void BluetoothCommunication::handler()
{
    connect_status();
    
    String bleData = receiveData();

    // Se recebeu algum dado...
    if (bleData.length() > 0)
    {
        
    } 
} 

String BluetoothCommunication::receiveData()
{
    String temp = receivedData;
    receivedData = "";
    return temp;
}

void BluetoothCommunication::processCommand(String command)
{
    command.trim();        // Remove espaços
    command.toUpperCase(); // Converte para maiúscula

    if (command == "START")
    {
        if (DEBUG_BLE)
            Serial.println("Comando: Iniciar movimento");
        // Inicia o carrinho W.I.P
    }
}

#endif