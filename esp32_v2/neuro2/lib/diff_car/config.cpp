
#define MOTOR_EN_A  42
#define MOTOR_IN1   40
#define MOTOR_IN2   39
#define MOTOR_IN3   38
#define MOTOR_IN4   37
#define MOTOR_EN_B  41

#define ENCODER_A_1 14
#define ENCODER_A_2 13
#define ENCODER_B_1 11
#define ENCODER_B_2 12

#define RFID_SDA 5
#define RFID_SCK 6
#define RFID_MOSI 7
#define RFID_MISO 4
#define RFID_RST 14
#define RFID_I2C_ADDR 0x28  // I2C address for MFRC522

#define GPS_RX 17
#define GPS_TX 16

#define SEN_REF_D1 36
#define SEN_REF_D2 35
#define SEN_REF_D3 45
#define SEN_REF_D4 48
#define SEN_REF_D5 47
#define SEN_REF_D6 21
#define SEN_REF_D7 20
#define SEN_REF_D8 19

#define MPU_9250_SCL 9
#define MPU_9250_SDA 8

#define EKF_N 5 // Position x and Angular z
#define EKF_M 5 // Velocity x and Angular z
#define _float_t double

#define NOBS 5
//#define ENCODER_QUAD TRUE
#define ENCODER_SIMPLE TRUE