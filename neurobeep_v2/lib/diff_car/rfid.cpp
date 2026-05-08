#include "diff_car.h"
#include <SPI.h>

MFRC522DriverPinSimple ss_pin(RFID_SDA);

MFRC522DriverSPI driver{ss_pin}; // Create SPI driver
//MFRC522DriverI2C driver{};     // Create I2C driver
MFRC522 mfrc522{driver};         // Create MFRC522 instance

void DiffCar::setup_rfid(){ 
    // Initialize SPI for RFID
    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
    

    Serial.println("Setting up RFID...");

    while (!Serial);       // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4).
  
    mfrc522.PCD_Init();    // Init MFRC522 board.
    MFRC522Debug::PCD_DumpVersionToSerial(mfrc522, Serial);	// Show details of PCD - MFRC522 Card Reader details.
    Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}
void DiffCar::update_rfid(){
    if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
    }

    // Select one of the cards.
    if (!mfrc522.PICC_ReadCardSerial()) {
        return;
    }

    // Dump debug info about the card; PICC_HaltA() is automatically called.
    // Store UID info in a DiffCar member variable instead of dumping to Serial
    this -> rfid_uid.clear();
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        this -> rfid_uid += String(mfrc522.uid.uidByte[i], HEX);
        if (i < mfrc522.uid.size - 1) this -> rfid_uid += ":";
    }
    Serial.print("RFID UID: ");
    Serial.println(this -> rfid_uid);
}









