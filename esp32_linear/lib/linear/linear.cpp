
#include "linear.h"

#include <stdio.h>
#include <math.h>


LinearCar::LinearCar() {}

void LinearCar::setup() {
  setup_mpu();
  setup_rfid();
  setup_gps();
}

// Função para enviar dados de telemetria via Bluetooth
