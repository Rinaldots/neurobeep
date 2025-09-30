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
  //diffCar.left_velocity_target = 0.3;
  //diffCar.right_velocity_target = 0.3;
}

int pwm_atual = 170;
long last_time = 0;

void loop() {
  long time = millis();
  diffCar.update_h_bridge();
  //diffCar.update_mpu();
  //diffCar.update_rfid();
  diffCar.velocity_update();
  //diffCar.handler_motor();
  //diffCar.debug_encoder();
  //diffCar.debug_line();
  Serial.print("Left actual: " + String(diffCar.left_velocity_ms) + " | Right actual: " + String(diffCar.right_velocity_ms));
  Serial.print(" | Left Motor PWM: "); Serial.print(diffCar.left_motor_pwm); Serial.print(" | Right Motor PWM: "); Serial.print(diffCar.right_motor_pwm);
  Serial.println();
  if(time - last_time > 5000){
    last_time = time;
    pwm_atual += 5;
    diffCar.left_motor_pwm = pwm_atual;
    diffCar.right_motor_pwm = pwm_atual;
    if(pwm_atual > 255) pwm_atual = 0;
  }
}

