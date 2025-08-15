#include <Arduino.h>
#include <diff_car.h>

// put function declarations here:
DiffCar diffCar;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  diffCar.setup_line_sensor(14,27,26,25,33,32,35,34,8);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t array[10];
  uint16_t line_position = diffCar.line_position(array);
  Serial.print("Sensor Values: ");
  for (int i = 0; i < 10; i++) {
    Serial.print(array[i]);
    Serial.print(" ");
  }
  Serial.print(line_position);
  Serial.println();
}

