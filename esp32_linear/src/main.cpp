#include <Arduino.h>
#include <linear.h>
#include <bluetooth.h>
// Debug timing (set to 1 to enable, 0 to disable)
#define ENABLE_TIMING 0

LinearCar linearCar;
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

	bluetooth.begin();
	
	linearCar.setup();

	xTaskCreatePinnedToCore(
		bleTask,           // Função da task
		"BLE_Task",        // Nome da task
		4096,              // Stack size (bytes)
		NULL,              // Parâmetro
		1,                 // Prioridade (1 = baixa, para não interferir)
		&bleTaskHandle,    // Handle da task
		0                  // Core 0 (Core 1 para loop principal)
	);
	linearCar.set_motor_speed(200);

	Serial.println("Setup complete. BLE task running on Core 0");
}

void loop() {
	//static unsigned long last_t = 0;

	//unsigned long now = micros();

	linearCar.update_h_bridge();

	
}

