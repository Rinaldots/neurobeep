// Configurações do RFID
#define RFID_SDA 5
#define RFID_SCK 6
#define RFID_MOSI 7
#define RFID_MISO 4
#define RFID_RST 14
#define RFID_I2C_ADDR 0x28  // I2C address for MFRC522

// Configurações do GPS
#define GPS_RX 17
#define GPS_TX 16

// Configurações do sensor de linha 
#define SEN_REF_D1 36
#define SEN_REF_D2 35
#define SEN_REF_D3 45
#define SEN_REF_D4 48
#define SEN_REF_D5 47
#define SEN_REF_D6 21
#define SEN_REF_D7 20
#define SEN_REF_D8 19

// Configurações do MPU9250
#define MPU_9250_SCL 9
#define MPU_9250_SDA 8

// Configurações do filtro de Kalman estendido
#define EKF_N 3
#define EKF_M 5
#define _float_t double

// Configurações do encoder
const int pulses_per_rev = 16*4;    // pulsos por volta do encoder (ajuste)
const float wheel_diameter_m = 0.065; // opcional para converter em m/s
const unsigned long DEBOUNCE_US = 3000; // rejeita pulsos < 3 ms (ajuste)
const unsigned long SAMPLE_MS = 100;    // janela principal de cálculo (100 ms)
const int MEDIAN_WINDOW = 5;            // janela mediana (odd)
const float EMA_ALPHA = 0.3f;           // 0..1, maior = resposta mais rápida
#define ENCODER_A_1 14
#define ENCODER_A_2 13
#define ENCODER_B_1 11
#define ENCODER_B_2 12

// Coeficientes para chute inicial dos motores
// Checar pasta controle para recalibrar

#define C1 -2.85738071e-05
#define C2 1.61248061e-02
#define C3 -1.85446691e+00
// Variavei PID
#define KP 0.5
#define KI 20.0
#define KD 10.0
// Variavei PID seguir linha
#define KP_l 0.0  
#define KI_l 35.0
#define KD_l 35.0



#define MIN_PULSES_PER_S 0.5   
#define NO_PULSE_TIMEOUT_MS 100
#define MOTOR_EN_A  42
#define MOTOR_IN1   40
#define MOTOR_IN2   39
#define MOTOR_IN3   38
#define MOTOR_IN4   37
#define MOTOR_EN_B  41

#define PULSES_PER_REV 10.0f
#define WHEEL_CIRCUMFERENCE_M 0.065f * 3.14159f

#define NOBS 5
#define ENCODER_QUAD TRUE
//#define ENCODER_SIMPLE TRUE