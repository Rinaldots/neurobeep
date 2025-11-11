#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include <string>

#define SERVICE_UUID        "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
#define CHARACTERISTIC_UUID "d5593e6b-3328-493a-b3c9-9814683d8e40"
#define WRITE_UUID "c1d5f8e4-3b2a-4f4e-9f2e-1a2b3c4d5e6f"
// Compile-time debug for BLE; set to 1 to enable verbose Serial prints
#ifndef DEBUG_BLE
#define DEBUG_BLE 0
#endif

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

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        if (DEBUG_BLE) Serial.println("Cliente BLE conectado");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        if (DEBUG_BLE) Serial.println("Cliente BLE desconectado");
        BLEDevice::startAdvertising(); // Restart advertising
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        // Read value into auto (std::string) then convert
        auto tmp = pCharacteristic->getValue();
        String rxValue = String(tmp.c_str());
        if (rxValue.length() > 0) {
            receivedData = rxValue;
            if (DEBUG_BLE) Serial.println("BLE onWrite received: " + rxValue);
        }
    }
};

class BluetoothCommunication { 
private:
    BLEServer* pServer;
    BLECharacteristic* pCharacteristic;
    BLEService* pService;
    // Cached telemetry: use compact binary format instead of text to reduce payload
    // Binary telemetry packet (header + data)
    uint8_t telemetry_cache_binary[120];
    size_t telemetry_cache_binary_len;
    
public:
    void begin();
    void sendData(const char* data);
    void updateTelemetryCache();
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
    
    if (DEBUG_BLE) Serial.println("O dispositivo BLE esta pronto para conexao");
}

void BluetoothCommunication::connect_status() {
    connected = deviceConnected;
}

void BluetoothCommunication::sendData(const char* data) {
    if (!(connected && pCharacteristic)) return;

    // send up to TELEMETRY_MAX_LEN bytes to avoid large packet fragmentation
    size_t len = strlen(data);
    if (len > TELEMETRY_PART_LEN) len = TELEMETRY_PART_LEN;

    // Use explicit setValue with length to avoid extra allocations
    pCharacteristic->setValue((uint8_t*)data, (size_t)len);
    pCharacteristic->notify();
}

void BluetoothCommunication::updateTelemetryCache() {
    // Build compact binary telemetry to reduce BLE payload and notify time
    // Format: [header:2][encoders:8][velocities:16][pwm:8][gains:8][odom_pos:12][odom_vel:12][imu:12][line:2][line_markers:6][gps:17][rfid:12]
    // Total: ~125 bytes 
    
    telemetry_cache_binary_len = 0;
    uint8_t* ptr = telemetry_cache_binary;
    
    // Header: 0xBEEF (magic number to identify binary format)
    *ptr++ = 0xBE; *ptr++ = 0xEF;
    
    // Encoders (2 x int32 counts)
    int32_t el = (int32_t)diffCar.encoder_left_count;
    int32_t er = (int32_t)diffCar.encoder_right_count;
    memcpy(ptr, &el, 4); ptr += 4;
    memcpy(ptr, &er, 4); ptr += 4;
    
    // Velocities (4 x float32: left, right, left_target, right_target)
    memcpy(ptr, &diffCar.left_velocity_ms, 4); ptr += 4;
    memcpy(ptr, &diffCar.right_velocity_ms, 4); ptr += 4;
    memcpy(ptr, &diffCar.left_velocity_target, 4); ptr += 4;
    memcpy(ptr, &diffCar.right_velocity_target, 4); ptr += 4;
    
    // PWM (2 x float32)
    memcpy(ptr, &diffCar.left_motor_pwm, 4); ptr += 4;
    memcpy(ptr, &diffCar.right_motor_pwm, 4); ptr += 4;
    
    // Gains (2 x float32)
    memcpy(ptr, &diffCar.left_gain, 4); ptr += 4;
    memcpy(ptr, &diffCar.right_gain, 4); ptr += 4;
    
    // Odometry position (3 x float32: x, y, theta from quaternion or yaw)
    float odom_x = diffCar.odometry.pose.position.x;
    float odom_y = diffCar.odometry.pose.position.y;
    float odom_theta = atan2(2.0*(diffCar.odometry.pose.orientation.w * diffCar.odometry.pose.orientation.z),
                             1.0 - 2.0*(diffCar.odometry.pose.orientation.z * diffCar.odometry.pose.orientation.z));
    memcpy(ptr, &odom_x, 4); ptr += 4;
    memcpy(ptr, &odom_y, 4); ptr += 4;
    memcpy(ptr, &odom_theta, 4); ptr += 4;
    
    // Odometry velocity (3 x float32: vx, vy, omega)
    float odom_vx = diffCar.odometry.vel.linear.x;
    float odom_vy = diffCar.odometry.vel.linear.y;
    float odom_omega = diffCar.odometry.vel.angular.z;
    memcpy(ptr, &odom_vx, 4); ptr += 4;
    memcpy(ptr, &odom_vy, 4); ptr += 4;
    memcpy(ptr, &odom_omega, 4); ptr += 4;
    
    // IMU accel (3 x float32)
    float accel_x = diffCar.mpu_accel.linear.x;
    float accel_y = diffCar.mpu_accel.linear.y;
    float accel_z = diffCar.mpu_accel.linear.z;
    memcpy(ptr, &accel_x, 4); ptr += 4;
    memcpy(ptr, &accel_y, 4); ptr += 4;
    memcpy(ptr, &accel_z, 4); ptr += 4;
    
    // Line sensor (1 x int16 distance mm)
    int16_t line_dist = diffCar.line_distance_mm;
    memcpy(ptr, &line_dist, 2); ptr += 2;
    
    // Line markers (1 x uint16 count + 1 x float32 distance = 6 bytes)
    memcpy(ptr, &diffCar.line_marker_count, 2); ptr += 2;
    memcpy(ptr, &diffCar.line_marker_distance_m, 4); ptr += 4;
    
    // GPS (4 x float32 + 1 x uint8 = 17 bytes)
    memcpy(ptr, &diffCar.gps_latitude, 4); ptr += 4;
    memcpy(ptr, &diffCar.gps_longitude, 4); ptr += 4;
    memcpy(ptr, &diffCar.gps_altitude, 4); ptr += 4;
    memcpy(ptr, &diffCar.gps_speed, 4); ptr += 4;
    *ptr++ = diffCar.gps_valid ? 1 : 0;
    
    // RFID: encode as 12-char string
    int rfid_len = diffCar.rfid_uid.length();
    if (rfid_len > 12) rfid_len = 12;
    for (int i = 0; i < rfid_len; i++) *ptr++ = diffCar.rfid_uid[i];
    for (int i = rfid_len; i < 12; i++) *ptr++ = 0; // pad with zeros
    
    telemetry_cache_binary_len = (size_t)(ptr - telemetry_cache_binary);
}

void BluetoothCommunication::handler() {
    connect_status();
    // Timing probes
#if DEBUG_BLE
    unsigned long t0 = micros();
#endif

    String bleData = receiveData();

#if DEBUG_BLE
    unsigned long t_after_receive = micros();
#endif

    if (bleData.length() > 0) {
        if (bleData.startsWith("CMD:")) {
            String command = bleData.substring(4);
            if (DEBUG_BLE) Serial.println("Comando recebido: " + command);

#if DEBUG_BLE
            unsigned long t_cmd_start = micros();
#endif
            processCommand(command);
#if DEBUG_BLE
            unsigned long t_cmd_end = micros();
            Serial.printf("[ble_timing] receive: %lu us | cmd_exec: %lu us\n", (unsigned long)(t_after_receive - t0), (unsigned long)(t_cmd_end - t_cmd_start));
#endif

        } else if (bleData.startsWith("RQS")) {
            // Telemetry request: send binary telemetry in a single BLE notification
            if (connected && telemetry_cache_binary_len > 0) {
#if DEBUG_BLE
                unsigned long t_notify_start = micros();
#endif
                // Send single binary message (much faster than 3 text messages)
                pCharacteristic->setValue(telemetry_cache_binary, telemetry_cache_binary_len);
                pCharacteristic->notify();

#if DEBUG_BLE
                unsigned long t_notify_end = micros();
                unsigned long notify_us = t_notify_end - t_notify_start;
                Serial.printf("[ble_timing] receive: %lu us | notify_binary(%d bytes): %lu us\n", 
                              (unsigned long)(t_after_receive - t0), telemetry_cache_binary_len, notify_us);
#endif
            }
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
        if (DEBUG_BLE) Serial.println("Comando: Iniciar movimento");
        // Inicia o carrinho W.I.P
    }
    else if (command == "RESET_KALMAN") {
        if (DEBUG_BLE) Serial.println("Comando: Reset Filtro de Kalman");
        diffCar.reset_kalman_filter();
    }
    else if (command == "CALIBRATE_IMU") {
        if (DEBUG_BLE) Serial.println("Comando: Calibrar IMU");
        diffCar.calibrate_imu();
    }
    else if (command == "CALIBRATE_LINE_SENSORS") {
        diffCar.calibrate_line_sensors();
    }
    else if (command.startsWith("VEL:")) {
        // Comando para definir velocidade: VEL:0.50 0.50 (esquerda direita)
        float left_velocity = command.substring(4, command.indexOf(' ', 4)).toFloat();
        float right_velocity = command.substring(command.indexOf(' ', 4) + 1).toFloat();
        if (DEBUG_BLE) Serial.println("Comando: Definir velocidade esquerda " + String(left_velocity) + ", direita " + String(right_velocity));
        diffCar.left_velocity_target = left_velocity;
        diffCar.right_velocity_target = right_velocity;
        diffCar.left_gain = 0.0;
        diffCar.right_gain = 0.0;
    }
    else if (command == "FOLLOW_LINE_START") {
        // Inicia seguidor de linha
        if (DEBUG_BLE) Serial.println("Comando: Iniciar seguidor de linha");
        diffCar.line_following_enabled = true;
    }
    else if (command == "FOLLOW_LINE_STOP") {
        // Para seguidor de linha
        if (DEBUG_BLE) Serial.println("Comando: Parar seguidor de linha");
        diffCar.line_following_enabled = false;
        diffCar.left_velocity_target = 0.0;
        diffCar.right_velocity_target = 0.0;
    }
    else if (command.startsWith("FOLLOW_LINE_CFG:")) {
        // Configura parâmetros do seguidor: FOLLOW_LINE_CFG:0.3 0.001 (base_speed kp)
        int space_idx = command.indexOf(' ', 16);
        if (space_idx > 0) {
            float base_speed = command.substring(16, space_idx).toFloat();
            float kp = command.substring(space_idx + 1).toFloat();
            if (DEBUG_BLE) Serial.printf("Comando: Config seguidor - velocidade: %.2f, kp: %.4f\n", base_speed, kp);
            diffCar.line_follow_base_speed = base_speed;
            diffCar.line_follow_kp = kp;
        }
    }
    else if (command == "SET_KP_KI_KD"){
        // Configura ganhos do PID: SET_KP_KI_KD:1.0 0.0 0.1 (kp ki kd)
        // Exemplo: SET_KP_KI_KD:1.0 0.0 0.1
        int first_space = command.indexOf(' ', 13);
        int second_space = command.indexOf(' ', first_space + 1);
        if (first_space > 0 && second_space > first_space) {
            float kp = command.substring(13, first_space).toFloat();
            float ki = command.substring(first_space + 1, second_space).toFloat();
            float kd = command.substring(second_space + 1).toFloat();
            if (DEBUG_BLE) Serial.printf("Comando: Config PID - kp: %.2f, ki: %.2f, kd: %.2f\n", kp, ki, kd);
            left_pid.SetTunings(kp, ki, kd);
            right_pid.SetTunings(kp, ki, kd);
        }
    }
    else if (command == "MARKER_RESET") {
        // Reseta contador de marcadores
        if (DEBUG_BLE) Serial.println("Comando: Reset marcadores de linha");
        diffCar.reset_line_markers();
    }
    else if (command.startsWith("MARKER_SPACING:")) {
        // Configura espaçamento entre marcadores: MARKER_SPACING:0.5
        float spacing = command.substring(15).toFloat();
        if (DEBUG_BLE) Serial.printf("Comando: Espaçamento marcadores = %.2f m\n", spacing);
        diffCar.marker_spacing_m = spacing;
    }
    else if (command.startsWith("MARKER_THRESHOLD:")) {
        // Configura threshold de detecção: MARKER_THRESHOLD:3500
        uint16_t threshold = command.substring(17).toInt();
        if (DEBUG_BLE) Serial.printf("Comando: Threshold marcadores = %d\n", threshold);
        diffCar.marker_threshold = threshold;
    }
    else {
        if (DEBUG_BLE) Serial.println("Comando desconhecido: " + command);
        // keep error sent so client knows command failed
        sendData("ERROR: Comando não reconhecido");
    }
}


#endif