#include "sensors.h"
#include "config.h"
#include <Arduino.h>

bool CONT_SENSOR_LINE_LEFT = false;
bool CONT_SENSOR_LINE_RIGHT = false;
bool CONT_SENSOR_LINE_CENTER = false;

// Timer para leitura periódica dos sensores analógicos
hw_timer_t * sensorTimer = NULL;

// Função de interrupção por timer para leitura analógica
void IRAM_ATTR onSensorTimer() {
    // Lê valores analógicos e atualiza variáveis
    int leftValue = analogRead(SENSOR_LINE_LEFT);
    int rightValue = analogRead(SENSOR_LINE_RIGHT);
    int centerValue = analogRead(SENSOR_LINE_CENTER);
    
    // Considera linha detectada se valor > 4090
    CONT_SENSOR_LINE_LEFT = (leftValue > 4094);
    CONT_SENSOR_LINE_RIGHT = (rightValue > 4094);
    CONT_SENSOR_LINE_CENTER = (centerValue > 4094);
}

void setupSensors() {
    // Configura os pinos como entrada analógica (não precisa de pull-up)
    pinMode(SENSOR_LINE_LEFT, INPUT);
    pinMode(SENSOR_LINE_RIGHT, INPUT);
    pinMode(SENSOR_LINE_CENTER, INPUT);
    
    // Configura timer para ler sensores a cada 10ms (100Hz)
    sensorTimer = timerBegin(1000000); // 1MHz frequency
    timerAttachInterrupt(sensorTimer, &onSensorTimer); // Anexa interrupção
    timerAlarm(sensorTimer, 10000, true, 0); // 10ms = 10000 microsegundos, repetir
    
    Serial.println("Sensors initialized with timer-based analog reading (100Hz).");
}

void readSensors() {
    Serial.print("Line Sensors - Left: ");
    Serial.print(CONT_SENSOR_LINE_LEFT);
    Serial.print(" | Center: ");
    Serial.print(CONT_SENSOR_LINE_CENTER);
    Serial.print(" | Right: ");
    Serial.println(CONT_SENSOR_LINE_RIGHT);
}