#include <Arduino.h>
#include <Adafruit_SSD1306.h>  // Adafruit SSD1306 (v2.5.7)

#include "global-config.h"     // Debug, Radio, Id
#include "pinout.h"            // Pinout


// *******
// I2C bus
// *******

TwoWire i2c = TwoWire(1);
boolean i2cStatus;

// ***************
// Battery voltage
// ***************

float vBat;

// ***********
// OLED module
// ***********

#define OLED_ADDR 0x3c  // I2C address
#define OLED_X 128      // Screen width in pixels
#define OLED_Y 64       // Screen height in pixels
#define OLED_LINE0 0    // Screen line 0 Y
#define OLED_LINE1 12   // Screen line 1 Y
#define OLED_LINE2 22   // Screen line 2 Y
#define OLED_LINE3 32   // Screen line 3 Y
#define OLED_LINE4 42   // Screen line 4 Y
#define OLED_LINE5 52   // Screen line 5 Y
Adafruit_SSD1306 oled(OLED_X, OLED_Y, &i2c, OLED_RST_PIN);

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

// ###############
// Setup functions
// ###############

// ----------
// Set up all
// ----------

void setup() {

  setupSerialDebug();
  setupVBat();
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

// ------------------
// Set up VBat sensor
// ------------------
void setupVBat() { pinMode(VBAT_ADC_PIN, INPUT); }

// --------------
// Set up I2C bus
// --------------
void setupI2c() {

    i2cStatus = i2c.begin(OLED_SDA, OLED_SCL);

    if (i2cStatus)
      Serial.println("I2C bus init ok");
    else
      Serial.println("I2C bus init failed");
}

// ------------------
// Set up OLED module
// ------------------
void setupOled() {
  
  if (i2cStatus)
    Serial.println("Oled init ok");
  else
    Serial.println("Oled init failed!");
  
  if (!i2cStatus) return;
    
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);

  oledUpdateLine(OLED_LINE0, TRACKER_NAME, false);
  oledUpdateLine(OLED_LINE4, "Charging...", false);
}


// ---------
// main loop
// ---------
void loop() {
  updateVBat();
  oledUpdateLine(OLED_LINE5, String(vBat) + " V", true);
  delay(5000);
}

// -----------------------
// Update VBat sensor data
// -----------------------
void updateVBat() { vBat = analogRead(VBAT_ADC_PIN) * 3.3 / 4096.0 / VBAT_DIVIDER; }

