#ifndef CONFIG_H
#define CONFIG_H


// Pinos dos sensores de linha
#define SENSOR_LINE_LEFT 13  // Defina o pino para o sensor de linha esquerdo
#define SENSOR_LINE_RIGHT  15// Defina o pino para o sensor de linha direito
#define SENSOR_LINE_CENTER 4

// Definição do pino onde o módulo de infravermelho está conectado
#define IR_SENSOR_PIN_left 5  // Pino digital para o sinal do sensor IR
#define IR_SENSOR_PIN_right 18  // Pino digital para o sinal do sensor IR

// Definições de pinos da ponte H
#define A_IA 32
#define A_IB 33
#define B_IA 25
#define B_IB 26
#define REVERSE_PIN 23
// Definições de pinos do encoder
#define EN_A 27
#define EN_B 14



// Definições de pinos PID padrao dos motores
const float DEFAULT_KP = 4.0f, DEFAULT_KI = 1.0f, DEFAULT_KD = 0.2f;

const int motor_startup = 800;

// Frequência e resolução do PWM
const int freq = 5000 , resolution = 10;
// Pulsos por revolução do encoder (verificar com seu encoder específico)
const float ENCODER_PULSES_PER_REVOLUTION = 20.0f;
// Distância entre os eixos das rodas (em metros)
const float wheels_y_distance_ = 0.16f; // Distância entre as rodas (em metros)
// Circunferência da roda (em metros)
const float wheel_circumference_ = 0.065f * 3.141;
const int time_update = 100;
#endif // CONFIG_H
