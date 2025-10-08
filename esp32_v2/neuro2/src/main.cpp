#include <Arduino.h>
#include <diff_car.h>
#include <bluetooth.h>
// put function declarations here:

DiffCar diffCar;
BluetoothCommunication bluetooth;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  ekf_t ekf = {0};
  bluetooth.begin();
  diffCar.calibrated = true; // pular calibração do sensor de linha

  diffCar.setup();
  Serial.println("Setup complete.");
}

void loop() {
  static unsigned long last_t = 0;
  unsigned long now = micros();

  if (now - last_t >= (unsigned long)(1000000 / 10)) { // 10 Hz
    last_t = now;
    diffCar.velocity_update();
    diffCar.update_mpu();
    diffCar.update_line_position();
    diffCar.odometry.update_raw_velocity(diffCar.left_velocity_ms, diffCar.right_velocity_ms, 0.16);
    diffCar.update_kalman_filter();
    diffCar.odometry.update_odometry(diffCar.velocity_x_est, diffCar.velocity_y_est, diffCar.angular_velocity_est, 0.16);
    //diffCar.debug_mpu();
    //diffCar.debug_encoder();
    //diffCar.odometry.debug();
    diffCar.reset_kalman_filter();
    diffCar.handler_motor();
    diffCar.update_rfid();
  }
  bluetooth.handler();
  //diffCr.update_h_bridge();
  //diffCar.debug_line();
  //diffCar.handler_motor();
  //diffCar.debug_encoder();
  //diffCar.debug_line();
}

