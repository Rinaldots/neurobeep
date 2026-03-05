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
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // Read value into auto (std::string) then convert
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
    BLECharacteristic *pCharacteristic;
    BLEService *pService;
    // Cached telemetry: use compact binary format instead of text to reduce payload
    // Binary telemetry packet (header + data)
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
    // 1. Header
    // O ESP32 é Little Endian. Para enviar [0xBE, 0xEF], escrevemos 0xEFBE.
    telemetry_data.header = 0xEFBE; 

    // 2. Encoders
    telemetry_data.encoder_left_count = (int32_t)linearCar.steps;
    telemetry_data.encoder_right_count = (int32_t)0;

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
    telemetry_data.odom_x = 0;
    telemetry_data.odom_y = 0;
    
    // Cálculo do Theta (Quaternion para Euler Yaw)
    double q_z = 0;
    double q_w = 0;
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

    // 10. RFID (String handling segura)
    // Limpa o array com zeros primeiro (padding)
    memset(telemetry_data.rfid_uid, 0, sizeof(telemetry_data.rfid_uid));
    // Copia a string (respeitando o limite de 12 bytes)
    // Se diffCar.rfid_uid for String do Arduino:
    // Se for std::string:
    // strncpy(telemetry_data.rfid_uid, diffCar.rfid_uid.c_str(), sizeof(telemetry_data.rfid_uid));
    telemetry_cache_binary_len = 0;
}

void BluetoothCommunication::handler()
{
    connect_status();
    
    // Timing probes
    #if DEBUG_BLE
    unsigned long t0 = micros();
    #endif

    String bleData = receiveData();

    #if DEBUG_BLE
    unsigned long t_after_receive = micros();
    #endif

    // Se recebeu algum dado...
    if (bleData.length() > 0)
    {
        bool sendTelemetryResponse = true; // Flag para controlar se envia resposta no final

        // CASO 1: Comandos Gerais ("CMD:...")
        if (bleData.startsWith("CMD:"))
        {
            String command = bleData.substring(4);
            if (DEBUG_BLE) Serial.println("Comando recebido: " + command);
            processCommand(command);
        }
        // CASO 2: Velocidade ("VEL:...")
        else if (bleData.startsWith("VEL:"))
        {
            processCommand(bleData);
        }
        // CASO 3: Requisição Explícita ("RQS")
        else if (bleData.startsWith("RQS"))
        {
            
        }
        // CASO 4: Genérico
        else 
        {
            if (DEBUG_BLE) Serial.println("Comando direto: " + bleData);
            processCommand(bleData);
        }

        // --- BLOCO UNIFICADO DE RESPOSTA ---
        // Envia a telemetria atualizada para o App sempre que processar um comando
        
        if (sendTelemetryResponse && connected && telemetry_cache_binary_len > 0)
        {
            #if DEBUG_BLE
            unsigned long t_notify_start = micros();
            #endif

            pCharacteristic->setValue((uint8_t*)&telemetry_data, sizeof(TelemetryData));
            pCharacteristic->notify();

            #if DEBUG_BLE
            unsigned long t_notify_end = micros();
            Serial.printf("[ble_timing] receive->process: %lu us | notify: %lu us\n",
                          (unsigned long)(t_after_receive - t0), 
                          (unsigned long)(t_notify_end - t_notify_start));
            #endif
        }
    } // Fecha if (bleData.length() > 0)
} // Fecha void handler()

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
        
    }
    else if (command == "RESET_KALMAN")
    {
        
    }
    else if (command == "CALIBRATE_IMU")
    {
        
    }
    else if (command == "CALIBRATE_LINE_SENSORS")
    {
        
    }
    else if (command.startsWith("VEL:"))
    {
        
    }
    else if (command == "FOLLOW_LINE_START")
    {
        linearCar.comando = PLAY;
    }
    else if (command == "FOLLOW_LINE_STOP")
    {
        linearCar.comando = STOP;
    }
    else if (command.startsWith("FOLLOW_LINE_CFG:"))
    {
        
    }
    else if (command == "SET_KP_KI_KD")
    {
        
    }
    else if (command == "MARKER_RESET")
    {
        
    }
    else if (command.startsWith("MARKER_SPACING:"))
    {
        
    }
    else if (command.startsWith("MARKER_THRESHOLD:"))
    {
        
    }
    else
    {
        if (DEBUG_BLE)
            Serial.println("Comando desconhecido: " + command);
        // keep error sent so client knows command failed
        sendData("ERROR: Comando não reconhecido");
    }
}

#endif