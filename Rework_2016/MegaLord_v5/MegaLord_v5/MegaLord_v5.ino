#include <MSGC_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>

Ublox_GPS GPS(&Serial3);
// Settings Array Breakdown - Default to "byte settingsArray[] = {0x07, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};" which gives you basic information at altitude
// settingsArray[0] = NavMode
// settingsArray[1] and settingsArray[2] = Data Rate (Update Frequency)
// settingsArray[3] and settingsArray[4] and settingsArray[5] = Baud Rate (Communication)
// settingsArray[6] = GLL messages Enable/Disable (0x00 is disabled)
// settingsArray[7] = GSA messages Enable/Disable(0x00 is disabled)
// settingsArray[8] = GSV messages Enable/Disable(0x00 is disabled)
// settingsArray[9] = RMC messages Enable/Disable(0x00 is disabled)
// settingsArray[10] = VTG messages Enable/Disable(0x00 is disabled)
byte settingsArray[] = {0x07, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01};
volatile long timecheck = 0;
boolean usingInterrupt = true;
volatile boolean SDConnect = false;     //boolean to control if SD card writing is available
volatile boolean inertialDetect = false; 
const int chipSelect = 10;              // for microSD card

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial2.begin(9600); //For Xbee
  Serial2.setTimeout(500); // Timeout for xbee connection
  Serial.begin(115200); //For Serial Monitor
  Serial.println("Booting MEGALORD");
  int xdeg;
  int ydeg;
  int zdeg;
  int orex;
  int orey;
  int orez;
  int Sys;
  int Gyro;
  int Accel;
  int Mag;



  
}

void loop() {
  // put your main code here, to run repeatedly:

}
