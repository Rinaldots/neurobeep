#ifndef CONFIG_H
#define CONFIG_H


// Pinos dos sensores de linha
#define SENSOR_LINE_LEFT 34  // Defina o pino para o sensor de linha esquerdo
#define SENSOR_LINE_RIGHT 35 // Defina o pino para o sensor de linha direito
#define SENSOR_LINE 21

// Definição do pino onde o módulo de infravermelho está conectado
#define IR_SENSOR_PIN_left 5  // Pino digital para o sinal do sensor IR
#define IR_SENSOR_PIN_right 18  // Pino digital para o sinal do sensor IR

// Número de pulsos por rotação (configurado para 20 pulsos por rotação)
#define PULSES_PER_REV 20  // Número de pulsos por rotação (configurado para 20 pulsos por rotação)

// Definições de pinos PWM dos motores
#define ENA_left 25       // Pino PWM (ENA) do motor esquerdo
#define ENB_right 33      // Pino PWM (ENB) do motor direito

// Frequência e resolução do PWM
#define PWM_FREQUENCY 5000  // Frequência do PWM (em Hz)
#define PWM_RESOLUTION 8    // Resolução do PWM (0-255)

// Canais PWM
#define ENA_left_PWM_CHANNEL 0  // Canal PWM para o motor esquerdo (ENA)
#define ENB_right_PWM_CHANNEL 1  // Canal PWM para o motor direito (ENB)

#endif // CONFIG_H
