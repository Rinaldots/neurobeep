#ifndef SENSORS_H
#define SENSORS_H

void setupSensors();
void readSensors();
extern bool CONT_SENSOR_LINE_LEFT;
extern bool CONT_SENSOR_LINE_RIGHT;
extern bool CONT_SENSOR_LINE_CENTER;
#endif // SENSORS_H