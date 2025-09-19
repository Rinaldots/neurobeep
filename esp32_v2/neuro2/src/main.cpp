#include <Arduino.h>
#include <diff_car.h>
#include <bluetooth.h>
// put function declarations here:

DiffCar diffCar;
BluetoothCommunication bluetooth;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  bluetooth.begin();
  diffCar.calibrated = true;
  diffCar.setup_mpu();
  diffCar.setup_h_bridge();
  diffCar.setup_rfid();
  diffCar.setup_line_sensor();
  diffCar.setup_encoder();
  diffCar.setup_timer();
  Serial.println("Setup complete.");
  diffCar.left_velocity_target = 0.2;
  diffCar.right_velocity_target = 0.2;
}



void loop() {
  diffCar.update_h_bridge();
  diffCar.update_mpu();
  diffCar.update_rfid();
  diffCar.velocity_update();
  diffCar.handler_motor();
  //diffCar.debug_encoder();
}

