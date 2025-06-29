#include "sensors.h"
#include "config.h"
#include <Arduino.h>
int CONT_SENSOR_LINE_LEFT = 0; 
int CONT_SENSOR_LINE_RIGHT = 0; 
int CONT_SENSOR_LINE_CENTER = 0;

void setupSensors() {

    Serial.println("Sensors initialized.");
}