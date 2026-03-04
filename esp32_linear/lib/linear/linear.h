#ifndef LINEAR_H
#define LINEAR_H

#include "config.cpp"

#include <Arduino.h>
#include "MPU9250.h"
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <QTRSensors.h>
#include <Preferences.h>


extern MPU9250 mpu;

 
struct Vector3{
    int x;
    int y;
    int z;
};

struct MpuData{
    Vector3 linear;
    Vector3 angular;
    Vector3 magnetic;
};

class LinearCar {
public:
    LinearCar();
    void setup();
    void update_h_bridge();
    void set_motor_speed(int speed);
    
private:
    void setup_mpu();
    void setup_rfid();
    void setup_gps();

    void loop_gps();
    void displayInfo();

    void correct_imu();
    void update_mpu();
    void calibrate_imu();
    void debug_mpu();
    void calibrate_gyro(uint8_t loops);
    void calibrate_accel(uint8_t loops);
    void calibrate_magnetometer(uint8_t loops);
    void pid_calibration(uint8_t readAddress, float kP, float kI, uint8_t loops);
    
    void setup_h_bridge();
    
    
    void step();

    void update_rfid();

    // Variáveis do motor de passo
    int passo = 0; // Entre 0 e 3 para controle do passo do motor
    bool direction = true; // true = forward, false = backward
    int motor_speed = 0; // Velocidade do motor em ms

    // Variável para armazenar o UID do RFID lido
    String rfid_uid;

    // GPS data
    float gps_latitude;
    float gps_longitude;
    float gps_altitude;
    bool gps_valid;
    float gps_speed;

    // MPU data
    MpuData mpu_accel;
    bool mpu_available = false;
};

extern LinearCar linear_car;

#endif