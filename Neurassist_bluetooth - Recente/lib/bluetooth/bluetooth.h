#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BluetoothSerial.h>
#include <Arduino.h>
void setupBluetooth();
int callbackBluetooth();
int motorHandler(int drive_mode); // drive_mode = 0 for RPM control, drive_mode = 1 for displacement control
#endif // BLUETOOTH_H