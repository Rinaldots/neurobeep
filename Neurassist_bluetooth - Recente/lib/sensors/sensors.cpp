#include "sensors.h"
#include "config.h"
#include <Arduino.h>

bool CONT_SENSOR_LINE_LEFT = false;
bool CONT_SENSOR_LINE_RIGHT = false;
bool CONT_SENSOR_LINE_CENTER = false;

void setupSensors() {
    pinMode(SENSOR_LINE_LEFT, INPUT);
    pinMode(SENSOR_LINE_RIGHT, INPUT);
    pinMode(SENSOR_LINE_CENTER, INPUT);
    Serial.println("Sensors initialized.");
}
void readSensors() {
    int TEMP_SENSOR_LINE_LEFT = analogRead(SENSOR_LINE_LEFT);
    int TEMP_SENSOR_LINE_RIGHT = analogRead(SENSOR_LINE_RIGHT);
    int TEMP_SENSOR_LINE_CENTER = analogRead(SENSOR_LINE_CENTER);
    if (TEMP_SENSOR_LINE_CENTER> 4090){
        CONT_SENSOR_LINE_CENTER = true;
    } else {
        CONT_SENSOR_LINE_CENTER = false;
    }
    if (TEMP_SENSOR_LINE_LEFT > 4090) {
        CONT_SENSOR_LINE_LEFT = true;
    } else {
        CONT_SENSOR_LINE_LEFT = false;
    }
    if (TEMP_SENSOR_LINE_RIGHT > 4090) {
        CONT_SENSOR_LINE_RIGHT = true;
    } else {
        CONT_SENSOR_LINE_RIGHT = false;
    }
    
    }