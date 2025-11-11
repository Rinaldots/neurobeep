#include <Arduino.h>
#include <diff_car.h>
#include <bluetooth.h>
// Debug timing (set to 1 to enable, 0 to disable)
#define ENABLE_TIMING 0

DiffCar diffCar;
BluetoothCommunication bluetooth;

// Task handle para a thread BLE
TaskHandle_t bleTaskHandle = NULL;

// Função da task BLE (roda em core separado)
void bleTask(void* parameter) {
	const TickType_t xDelay = pdMS_TO_TICKS(10); // 10ms = ~100Hz max rate
	
	for (;;) {
		// Atualiza cache de telemetria no contexto da task BLE
		bluetooth.updateTelemetryCache();
		bluetooth.handler();
		vTaskDelay(xDelay);
	}
}

void setup() {
	Serial.begin(115200);
	Serial.println("Starting...");
	ekf_t ekf = {0};
	bluetooth.begin();
	
	diffCar.calibrated = false; 
	diffCar.setup();
	//diffCar.calibrate_lines();
	//delay(1500);
	// Cria task BLE em Core 0 (Core 1 fica para o loop principal)
	// Prioridade 1, stack 4096 bytes
	xTaskCreatePinnedToCore(
		bleTask,           // Função da task
		"BLE_Task",        // Nome da task
		4096,              // Stack size (bytes)
		NULL,              // Parâmetro
		1,                 // Prioridade (1 = baixa, para não interferir)
		&bleTaskHandle,    // Handle da task
		0                  // Core 0 (Core 1 para loop principal)
	);
	
	Serial.println("Setup complete. BLE task running on Core 0");
}

void loop() {
	static unsigned long last_t = 0;
	static unsigned long last_t2 = 0;
	unsigned long now = micros();

	if(now - last_t >= (unsigned long)(1000000 / 100)) { // 100 Hz
		last_t2 = now;
		// 1) velocity update
		diffCar.velocity_update();
		#if ENABLE_TIMING
			d_velocity = micros() - t_prev; t_prev = micros();
		#endif
		// 8) motor handler
		diffCar.handler_motor();
		#if ENABLE_TIMING
			d_motor = micros() - t_prev; t_prev = micros();
		#endif
	}
	if (now - last_t >= (unsigned long)(1000000 / 40)) { // 40 Hz
		last_t = now;
		unsigned long t0 = micros();

		#if ENABLE_TIMING
			unsigned long t_prev = t0;
			unsigned long d_velocity = 0, d_mpu = 0, d_line = 0, d_rawvel = 0;
			unsigned long d_kalman = 0, d_odom = 0, d_reset = 0, d_motor = 0, d_rfid = 0, d_bt = 0;
		#endif
		
		// 2) mpu
		diffCar.update_mpu();
		#if ENABLE_TIMING
			d_mpu = micros() - t_prev; t_prev = micros();
		#endif
		// 3) line sensor
		diffCar.update_line_position();
		#if ENABLE_TIMING
			d_line = micros() - t_prev; t_prev = micros();
		#endif
		/*
		// 4) raw velocity -> odometry helper
		diffCar.odometry.update_raw_velocity(diffCar.left_velocity_ms, diffCar.right_velocity_ms, 0.16);
		#if ENABLE_TIMING
			d_rawvel = micros() - t_prev; t_prev = micros();
		#endif
		// 5) kalman update
		diffCar.update_kalman_filter();
		#if ENABLE_TIMING
			d_kalman = micros() - t_prev; t_prev = micros();
		#endif
		// 6) odometry update
		diffCar.odometry.update_odometry(diffCar.velocity_x_est, diffCar.velocity_y_est, diffCar.angular_velocity_est, 0.16);
		#if ENABLE_TIMING
			d_odom = micros() - t_prev; t_prev = micros();
		#endif
		// 7) reset kalman (if used)
		diffCar.reset_kalman_filter();
		#if ENABLE_TIMING
			d_reset = micros() - t_prev; t_prev = micros();
		#endif
		*/
		
		
		// 8.5) Line follower (se ativado)
		if (diffCar.line_following_enabled) {
			diffCar.follow_line(diffCar.line_follow_base_speed, diffCar.line_follow_kp);
		}
		
		// 9) rfid @ 5 Hz (every 200 ms) - evita que RFID bloqueie o loop principal
		static unsigned long last_rfid_time = 0;
		const unsigned long RFID_INTERVAL_US = 200000UL; // 200 ms -> 5 Hz
		if ((unsigned long)(micros() - last_rfid_time) >= RFID_INTERVAL_US) {
			diffCar.update_rfid();
		#if ENABLE_TIMING
				d_rfid = micros() - t_prev; t_prev = micros();
		#endif
				last_rfid_time = micros();
			} else {
		#if ENABLE_TIMING
				d_rfid = 0;
		#endif
			}

		// optional debug helpers (commented out)
		//diffCar.debug_mpu();
		//diffCar.debug_encoder();
		//delay(100);
		//diffCar.odometry.debug();
		//diffCar.debug_line();


		
		#if ENABLE_TIMING
			auto print_ms = [&](const char* name, unsigned long us){
				float ms = us / 1000.0f;
				Serial.printf("%s: %.3f ms | ", name, ms);
			};

			Serial.print("[timing] ");
			print_ms("vel", d_velocity);
			print_ms("mpu", d_mpu);
			print_ms("line", d_line);
			print_ms("rawv", d_rawvel);
			print_ms("kal", d_kalman);
			print_ms("odom", d_odom);
			print_ms("rst", d_reset);
			print_ms("mot", d_motor);
			print_ms("rfid", d_rfid);
			Serial.println();
		#endif
		}

}

