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

// ***********
// OLED module
// ***********

#define OLED_ADDR 0x3c  // I2C address
#define OLED_X 128      // Screen width in pixels
#define OLED_Y 64       // Screen height in pixels
#define OLED_LINE0 0    // Screen line 0 Y
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
  setupGPS();
  setupI2c();
  setupOled();
}

// --------------------
// Sets up debug serial
// --------------------
void setupSerialDebug() {

  Serial.begin(SERIAL_DEBUG_BAUD_RATE);
  delay(2500);  // Give time to the serial monitor to pick up
}

// -----------------
// Set up GPS module
// -----------------
void setupGPS() {

  GPS_SERIAL.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(GPS_ON_PIN, OUTPUT);
  digitalWrite(GPS_ON_PIN, HIGH);
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
