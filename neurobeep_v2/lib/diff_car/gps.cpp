#include "diff_car.h"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of HardwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins defined in config.
*/
static const int RXPin = GPS_TX, TXPin = GPS_RX;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

HardwareSerial gpsSerial(2);

void DiffCar::setup_gps(){
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void DiffCar::loop_gps(){
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void DiffCar::displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    // Salvar dados GPS nas variáveis da classe
    gps_latitude = gps.location.lat();
    gps_longitude = gps.location.lng();
    gps_valid = true;
    
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    gps_valid = false;
    Serial.print(F("INVALID"));
  }

  // Salvar altitude se disponível
  if (gps.altitude.isValid())
  {
    gps_altitude = gps.altitude.meters();
  }
  
  // Salvar velocidade se disponível
  if (gps.speed.isValid())
  {
    gps_speed = gps.speed.kmph();
  }

  // Add satellite information for debugging
  Serial.print(F("  Satellites: "));
  if (gps.satellites.isValid())
  {
    Serial.print(gps.satellites.value());
  }
  else
  {
    Serial.print(F("NONE"));
  }
  Serial.print(F("  HDOP: "));
  if (gps.hdop.isValid())
  {
    Serial.print(gps.hdop.hdop(), 2);
  }
  else
  {
    Serial.print(F("N/A"));
  }
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Age: "));
  Serial.print(gps.location.age());
  Serial.print(F("ms"));

  Serial.println();
}
