#include <Arduino.h>
#include <diff_car.h>

// put function declarations here:

DiffCar diffCar;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  //diffCar.setup_line_sensor();
  //diffCar.setup_encoder();
  diffCar.calibrated = true;
  diffCar.setup_mpu();
  diffCar.setup_h_bridge();
  diffCar.setup_rfid();
  diffCar.setup_line_sensor();
  diffCar.setup_encoder();
  diffCar.setup_timer();
  Serial.println("Setup complete.");
  diffCar.set_motor_speed(0, 0);
}

void loop() {
  
  diffCar.update_h_bridge();
  diffCar.update_mpu();
  diffCar.update_rfid();
  diffCar.velocity_update();  // Safe to call here (not in ISR)
  diffCar.debug_encoder();
  //diffCar.debug_mpu();
  //diffCar.update_line_position();
  //diffCar.debug_line();
  
}

