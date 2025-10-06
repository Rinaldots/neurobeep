#include "diff_car.h"
#include <cmath>



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

int16_t DiffCar::line_dist_center_mm() {
    float line_position = (float)this->line_position_value / 1000.0f;
    return (int16_t)(line_position* 100.0f - 350.0f);  // Converter para mm e centralizar em zero
}


void DiffCar::update_line_position() {
    uint16_t line_position = qtr.readLineBlack(this -> line_sensor_array);
    int16_t xcentro_mm = line_dist_center_mm();  // Usar versão inteira
    
    this->line_distance_mm = xcentro_mm;
    
    this->light_sensor.orientation.z = atan2(xcentro_mm, 15000);
    
    this->line_position_value = line_position;
}

void DiffCar::debug_line() {
    Serial.print("Line sensor readings: ");
    
    for (int i = 0; i < 8; i++) {
        Serial.print(this->line_sensor_array[i],4);
        if (i < 7) Serial.print(", ");
    }
    
    Serial.print(" | Position: ");
    Serial.print(this->line_position_value);
    Serial.print(" | Dist from center (mm): ");
    Serial.print(this->line_distance_mm); 
    Serial.print(" | Angle (rad): ");
    Serial.print(this->light_sensor.orientation.z);
    
    Serial.println();
}
