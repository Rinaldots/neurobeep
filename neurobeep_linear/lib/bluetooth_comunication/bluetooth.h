#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

// UUID do Serviço Principal
#define SERVICE_UUID "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"

// CANO RX (Onde o ESP32 RECEBE os comandos do PC/React) - Mantivemos o seu antigo!
#define CHAR_RX_UUID "d5593e6b-3328-493a-b3c9-9814683d8e40"

// CANO TX (Onde o ESP32 ENVIA os dados pro PC/React) - UUID NOVO!
#define CHAR_TX_UUID "12345678-1234-5678-1234-56789abcdef0"

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
    uint32_t esp_timestamp_ms;      // 115 - 4
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
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // Lê do cano RX
        auto tmp = pCharacteristic->getValue();
        String rxValue = String(tmp.c_str());
        if (rxValue.length() > 0)
        {
            receivedData = rxValue;
            //if (DEBUG_BLE)
                //Serial.println("BLE onWrite received: " + rxValue);
        }
    }
};

class BluetoothCommunication
{
private:
    BLEServer *pServer;
    BLECharacteristic *pCharTX; // Criado ponteiro dedicado para envio
    BLECharacteristic *pCharRX; // Criado ponteiro dedicado para recebimento
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

    // 1. CARACTERÍSTICA TX (ESP32 envia Notificações para o PC)
    pCharTX = pService->createCharacteristic(
        CHAR_TX_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pCharTX->addDescriptor(new BLE2902());

    // 2. CARACTERÍSTICA RX (ESP32 recebe Comandos do PC)
    pCharRX = pService->createCharacteristic(
        CHAR_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | 
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharRX->setCallbacks(new MyCallbacks());

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();

    if (DEBUG_BLE)
        Serial.println("O dispositivo BLE esta pronto para conexao com TX e RX separados");
}

void BluetoothCommunication::connect_status()
{
    connected = deviceConnected;
}

void BluetoothCommunication::sendData(const char *data)
{
    if (!(connected && pCharTX))
        return;

    size_t len = strlen(data);
    if (len > TELEMETRY_PART_LEN)
        len = TELEMETRY_PART_LEN;

    pCharTX->setValue((uint8_t *)data, (size_t)len);
    pCharTX->indicate();
}

void BluetoothCommunication::updateTelemetryCache()
{
    // 1. Header
    telemetry_data.header = 0xEFBE;

    // 2. Encoders
    telemetry_data.encoder_left_count = 100;
    telemetry_data.encoder_right_count = 100;

    // 3. Velocities
    telemetry_data.left_velocity_ms = 0;
    telemetry_data.right_velocity_ms = 0;
    telemetry_data.left_velocity_target = 0;
    telemetry_data.right_velocity_target = 0;

    // 4. PWM & Gains
    telemetry_data.left_motor_pwm = 0;
    telemetry_data.right_motor_pwm = 0;
    telemetry_data.left_gain = 0;
    telemetry_data.right_gain = 0;

    // 5. Odometry Position
    telemetry_data.odom_x = 100;
    telemetry_data.odom_y = 0;
    telemetry_data.odom_theta = 0;

    // 6. Odometry Velocity
    telemetry_data.odom_vx = 0;
    telemetry_data.odom_vy = 0;
    telemetry_data.odom_omega = 0;

    // 7. IMU
    telemetry_data.accel_x = 0;
    telemetry_data.accel_y = 0;
    telemetry_data.accel_z = 0;

    // 8. Line Sensors
    telemetry_data.line_distance_mm = 0;
    telemetry_data.line_marker_count = 0;
    telemetry_data.line_marker_distance_m = 0;

    // 9. GPS
    telemetry_data.gps_latitude = 0;
    telemetry_data.gps_longitude = 0;
    telemetry_data.gps_altitude = 0;
    telemetry_data.gps_speed = 0;
    telemetry_data.gps_valid = 0;

    // 10. RFID
    memset(telemetry_data.rfid_uid, 0, sizeof(telemetry_data.rfid_uid));

    telemetry_cache_binary_len = sizeof(TelemetryData);
}

void BluetoothCommunication::handler()
{
    connect_status();
    
    #if DEBUG_BLE
    unsigned long t0 = micros();
    #endif

    String bleData = receiveData();

    #if DEBUG_BLE
    unsigned long t_after_receive = micros();
    #endif

    // --- 1. BLOCO DE RECEBIMENTO DE COMANDOS ---
    if (bleData.length() > 0)
    {
        if (bleData.startsWith("CMD:"))
        {
            String command = bleData.substring(4);
            if (DEBUG_BLE) Serial.println("Comando recebido: " + command);
            processCommand(command);
        }
        else if (bleData.startsWith("VEL:"))
        {
            processCommand(bleData);
        }
        else if (bleData.startsWith("RQS"))
        {
            if (DEBUG_BLE) Serial.println("Requisição explícita de telemetria recebida");
            updateTelemetryCache();

            // COLOQUEI O TESTE AQUI! Se o pacote de 115 bytes não for, descomente a linha abaixo e comente a struct
            // String teste = "BEEF_OK";
            // pCharTX->setValue((uint8_t*)teste.c_str(), teste.length());
            telemetry_data.esp_timestamp_ms = millis();
            pCharTX->setValue((uint8_t*)&telemetry_data, sizeof(TelemetryData));
            pCharTX->indicate();
            Serial.println("Enviado" + String(sizeof(TelemetryData)) + " bytes de telemetria em resposta ao RQS");
        }
        else 
        {
            if (DEBUG_BLE) Serial.println("Comando direto: " + bleData);
            processCommand(bleData);
        }
    } 

    // --- 2. BLOCO DE ENVIO CONTÍNUO DE TELEMETRIA ---
    static unsigned long lastTelemetryTime = 0;
    unsigned long currentMillis = millis();

    // REMOVI o "&& false" temporariamente para focar no RQS que vc está testando
    // Mudei para 300ms só para o buffer não engasgar durante os testes
    if (connected && (currentMillis - lastTelemetryTime >= 300)) 
    {
        lastTelemetryTime = currentMillis;

        updateTelemetryCache();

        if (telemetry_cache_binary_len > 0)
        {   telemetry_data.esp_timestamp_ms = millis();
            pCharTX->setValue((uint8_t*)&telemetry_data, sizeof(TelemetryData));
            pCharTX->indicate();
        }
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
    command.trim();
    command.toUpperCase();

    if (command == "START")
    {
        if (DEBUG_BLE)
            Serial.println("Comando: Iniciar movimento");
    }
    else if (command.startsWith("VEL:"))
    {
        
    }
    else if (command == "FOLLOW_LINE_START")
    {
        if (DEBUG_BLE)
            Serial.println("Comando: Iniciar seguidor de linha");
    }
    else if (command == "FOLLOW_LINE_STOP")
    {
        if (DEBUG_BLE)
            Serial.println("Comando: Parar seguidor de linha");
    }
}

#endif