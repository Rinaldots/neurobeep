#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "config.h"
#include "motor_controle.h"
#include "odometry.h"
#include <BluetoothSerial.h>
#include <Arduino.h>
void setupBluetooth();
int callbackBluetooth();

#endif // BLUETOOTH_H