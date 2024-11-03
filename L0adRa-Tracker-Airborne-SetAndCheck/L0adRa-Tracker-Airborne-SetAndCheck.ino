#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>  // Adafruit SSD1306 (v2.5.7)
#include "global-config.h"     // Debug, Radio, Id
#include "pinout.h"            // Pinout

// *******
// I2C bus
// *******
boolean i2cStatus;
TwoWire i2c = TwoWire(1);


// **********
// GPS module
// **********

#define GPS_SERIAL Serial2  // serial port
#define GPS_BAUD_RATE 9600  // baudrate

const byte SET_AIRBORNE1G[44] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 
                                  0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 
                                  0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 
                                  0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                  0x00, 0x00, 0x16, 0xDC};
const byte GET_AIRBORNE[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};

const byte NMEA_GGA_OFF[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
const byte NMEA_GGA_ON[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28};

const byte NMEA_GLL_OFF[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
const byte NMEA_GLL_ON[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2F};

const byte NMEA_GSA_OFF[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
const byte NMEA_GSA_ON[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x36};

const byte NMEA_GSV_OFF[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
const byte NMEA_GSV_ON[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3D};

const byte NMEA_RMC_OFF[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
const byte NMEA_RMC_ON[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x44};

const byte NMEA_VTG_OFF[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
const byte NMEA_VTG_ON[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0x4B};

byte UBX_RESPONSE[54];

#define UBX_DELAY 250
// ***********
// OLED module
// ***********

#define OLED_ADDR 0x3c  // I2C address
#define OLED_X 128      // Screen width in pixels
#define OLED_Y 64       // Screen height in pixels
#define OLED_LINE0 0    // Screen line 0 Y
#define OLED_LINE1 12    // Screen line 0 Y
#define OLED_LINE2 22   // Screen line 2 Y
Adafruit_SSD1306 oled(OLED_X, OLED_Y, &i2c, OLED_RST_PIN);
boolean oledStatus;

// ###############
// Setup functions
// ###############

// ----------
// Set up all
// ----------

void setup() {

  setupSerialDebug();
  delay(2500); // Give time to the serial monitor to pick up
  setupGPS();
  delay(1000);
  setupI2c();
  setupOled();

  oledUpdateLine(OLED_LINE1, "muting NMEA", true);
  gpsSentencesMute();
  
  oledUpdateLine(OLED_LINE1, "reading AirBorne", true);  
  int airborne = getGPSAirborne();
  //Serial.write(UBX_RESPONSE, 54);
  String airborneString = "";
  if (airborne != 6) 
    airborneString = "Not airborne :-(";
  else
    airborneString = "Airborne :-)";
  oledUpdateLine(OLED_LINE2, airborneString, true);  
  delay(UBX_DELAY);

  oledUpdateLine(OLED_LINE1, "switching airBorne", true);  
  sendGPSCommand(SET_AIRBORNE1G, 44);
  delay(UBX_DELAY);
  
  oledUpdateLine(OLED_LINE1, "reading AirBorne", true);  
  airborne = getGPSAirborne();
  Serial.write(UBX_RESPONSE, 54);
  if (airborne != 6) 
    airborneString = "Not airborne :-(";
  else
    airborneString = "Airborne :-)";
  oledUpdateLine(OLED_LINE2, airborneString, true);  
  delay(UBX_DELAY);

  oledUpdateLine(OLED_LINE1, "unmuting NMEA", true);
  gpsSentencesUnmute();
  oledUpdateLine(OLED_LINE1, "", true);
}

// --------------------
// Sets up debug serial
// --------------------
void setupSerialDebug() {

  Serial.begin(SERIAL_DEBUG_BAUD_RATE);
}

// -----------------
// Set up GPS module
// -----------------
void setupGPS() {

  GPS_SERIAL.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(GPS_ON_PIN, OUTPUT);
  digitalWrite(GPS_ON_PIN, HIGH);
}

void gpsSentencesMute() {
  sendGPSCommand(NMEA_GGA_OFF, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_GLL_OFF, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_GSA_OFF, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_GSV_OFF, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_RMC_OFF, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_VTG_OFF, 16);
  delay(UBX_DELAY); 
}

void gpsSentencesUnmute() {
  sendGPSCommand(NMEA_GGA_ON, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_GSV_ON, 16);
  delay(UBX_DELAY);
  sendGPSCommand(NMEA_RMC_ON, 16);
  delay(UBX_DELAY); 
}

void sendGPSCommand(const unsigned char *command, int size) {
  Serial2.write(command, size);
}

int getGPSAirborne() {
  while (GPS_SERIAL.available()) {
    GPS_SERIAL.read();
  }
  sendGPSCommand(GET_AIRBORNE, 8);
  int index = 0;
  delay(UBX_DELAY);
  while (GPS_SERIAL.available()) {
    UBX_RESPONSE[index++] = GPS_SERIAL.read();
  }
  return UBX_RESPONSE[8];

}

// --------------
// Set up I2C bus
// --------------
void setupI2c() { i2cStatus = i2c.begin(OLED_SDA, OLED_SCL); }

// ------------------
// Set up OLED module
// ------------------
void setupOled() {

  oledStatus = i2cStatus;    
  if (!oledStatus) return;
    
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);

  oled.setCursor(0, OLED_LINE0);
  oled.print(TRACKER_NAME);
  oled.display();
}

// ---------------------
// Update a line of text
// ---------------------
// line: line number
// message: message to display
// eraseBefore: true if line has to be erased first
// ----------------------
void oledUpdateLine(unsigned int line, String message, boolean eraseBefore) {

  if (eraseBefore) {
    oled.fillRect(0, line, OLED_X, 8, BLACK);
    oled.display();
  }
  oled.setCursor(0, line);
  oled.print(message);
  oled.display();
}

// ---------
// main loop
// ---------
void loop() {

  if (Serial.available()) {        
    GPS_SERIAL.write(Serial.read()); 
  }

  if (GPS_SERIAL.available()) {       // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(GPS_SERIAL.read());  // read it and send it out Serial (USB)
  }
}
