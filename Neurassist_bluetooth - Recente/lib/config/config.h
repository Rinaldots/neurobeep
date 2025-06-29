#ifndef CONFIG_H
#define CONFIG_H


// Pinos dos sensores de linha
#define SENSOR_LINE_LEFT 23  // Defina o pino para o sensor de linha esquerdo
#define SENSOR_LINE_RIGHT 13 // Defina o pino para o sensor de linha direito
#define SENSOR_LINE_CENTER 15

// Definição do pino onde o módulo de infravermelho está conectado
#define IR_SENSOR_PIN_left 5  // Pino digital para o sinal do sensor IR
#define IR_SENSOR_PIN_right 18  // Pino digital para o sinal do sensor IR

// Definições de pinos da ponte H
#define A_IA 32
#define A_IB 33
#define B_IA 25
#define B_IB 26
// Definições de pinos do encoder
#define EN_A 27
#define EN_B 14



// Definições de pinos PID padrao dos motores
const float DEFAULT_KP = 2.0f, DEFAULT_KI = 0.4f, DEFAULT_KD = 0.002f, DEFAULT_MIN_PWM = 800.0f;
// Frequência e resolução do PWM
const int freq = 5000 , resolution = 10;
// Pulsos por revolução do encoder
const float ENCODER_PULSES_PER_REVOLUTION = 16.0f;
// Distância entre os eixos das rodas (em metros)
const float wheels_y_distance_ = 0.16f; // Distância entre as rodas (em metros)
// Circunferência da roda (em metros)
const float wheel_circumference_ = 0.065f * 3.141;

#endif // CONFIG_H
