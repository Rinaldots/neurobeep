#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BluetoothSerial.h>
#include <Arduino.h>
void setupBluetooth();
int callbackBluetooth();
int motorHandler(int drive_mode); 
void readSensors(); 
#endif // BLUETOOTH_H