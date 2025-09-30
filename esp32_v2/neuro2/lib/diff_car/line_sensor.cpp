#include "diff_car.h"



void DiffCar::setup_line_sensor(){
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){(uint8_t)SEN_REF_D1, (uint8_t)SEN_REF_D2, (uint8_t)SEN_REF_D3, (uint8_t)SEN_REF_D4, (uint8_t)SEN_REF_D5, (uint8_t)SEN_REF_D6, (uint8_t)SEN_REF_D7, (uint8_t)SEN_REF_D8}, 8);

    delay(100);
    if (!this->calibrated) {
        Serial.print("Calibrating...");
        for (uint16_t i = 0; i < 400; i++)
        {
            qtr.calibrate();
        }
        Serial.println(" Done!");
        this->calibrated = true;
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(qtr.calibrationOn.minimum[i]);
            Serial.print(' ');
        }
        Serial.println();

        // print the calibration maximum values measured when emitters were on
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(qtr.calibrationOn.maximum[i]);
            Serial.print(' ');
        }
    }
    Serial.println();
    Serial.println();
    Serial.println("Line sensor setup complete.");
}
void DiffCar::update_line_position() {
    uint16_t line_position = qtr.readLineBlack(this -> line_sensor_array);
    this -> line_position_value = line_position;
}
void DiffCar::debug_line() {
    Serial.print("Line sensor readings: ");
    
    for (int i = 0; i < 8; i++) {
        Serial.print(this->line_sensor_array[i]);
        if (i < 7) Serial.print(", ");
    }
    
    
    Serial.println();
}