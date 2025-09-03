void DiffCar::setup_line_sensor(int sensor_a1, int sensor_a2, int sensor_a3, int sensor_a4, int sensor_a5, int sensor_a6, int sensor_a7, int sensor_a8, int SensorCount){
    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){(uint8_t)sensor_a1, (uint8_t)sensor_a2, (uint8_t)sensor_a3, (uint8_t)sensor_a4, (uint8_t)sensor_a5, (uint8_t)sensor_a6, (uint8_t)sensor_a7, (uint8_t)sensor_a8}, SensorCount);
    

    delay(100);
    Serial.print("Calibrating...");
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    Serial.println(" Done!");
}



void DiffCar::update_line_sensor(uint16_t* array) {
    qtr.readLineBlack(array);
}

uint16_t DiffCar::line_position(uint16_t* array) {
    uint16_t line_position = qtr.readLineBlack(array);
    return line_position;
}