#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>                   // Arduino/ Sparkfun 1.2.3 -> 1.3.4 (https://docs.arduino.cc/libraries/sd/)

#include <ESP32Servo.h>

#include "lmic.h"               // MCCI LoRaWAN LMIC by IBM, ... (v4.1.1)
#include <hal/hal.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>  // Adafruit SSD1306 (v2.5.9) -> 2.5.12
#include <DHT.h>               // DHT Sensor Library by Adafruit (v1.4.4)
#include <Adafruit_BMP085.h>   // Adafruit BMP085 Library (v1.2.2)
#include <DFRobot_SCD4X.h>     // DFRobot SCD4x Library (v1.0.1)
#include <TinyGPS.h>           // TinyGPS by Mikal Hart (v13.0.0) 
                               // N.B. for multi-constellations lib's .c must be patched 
                               // to consider GNxxx sentences instead of GPxxx ones**

#include "global-config.h"     // Debug, Radio, Id
#include "pinout.h"            // Pinout
#include "LoRaWan-ABP-auth.h"  // LoRaWan ABP authentication keys

// ##############################################
// Sensor/modules related constants and variables
// ##############################################

// *******
// I2C bus
// *******
boolean i2cStatus;
TwoWire i2c = TwoWire(1);

// *******
// SysTime
// *******

#define SYSTIME_PAYLOAD_FLAG_OFFSET 0
#define SYSTIME_PAYLOAD_FLAG_BIT 7

// *******************
// DHT temp/hum sensor
// *******************

#define DHT_TYPE DHT22  // either DHT11 or DHT22
#define DHT_TEMP_MIN_VALUE -74.0
#define DHT_TEMP_MAX_VALUE 54.0
#define DHT_TEMP_ACCURACY 0.5
#define DHT_TEMP_PAYLOAD_FLAG_OFFSET 0
#define DHT_TEMP_PAYLOAD_FLAG_BIT 6
#define DHT_HUMIDITY_MIN_VALUE 0.0
#define DHT_HUMIDITY_MAX_VALUE 100.0
#define DHT_HUMIDITY_ACCURACY 1.0
#define DHT_HUMIDITY_PAYLOAD_FLAG_OFFSET 0
#define DHT_HUMIDITY_PAYLOAD_FLAG_BIT 5

DHT dht(DHT_PIN, DHT_TYPE);
float dhtTempValue;
float dhtHumidityValue;

// ***************************
// BMP180 pressure/temp sensor
// ***************************

Adafruit_BMP085 bmp;
boolean bmpStatus;
float bmpTempValue;
unsigned int bmpPressureValue;

#define BMP_TEMP_MIN_VALUE -74.0
#define BMP_TEMP_MAX_VALUE 54.0
#define BMP_TEMP_ACCURACY 0.5
#define BMP_TEMP_PAYLOAD_FLAG_OFFSET 0
#define BMP_TEMP_PAYLOAD_FLAG_BIT 4
#define BMP_PRESSURE_MIN_VALUE 0
#define BMP_PRESSURE_MAX_VALUE 120000
#define BMP_PRESSURE_ACCURACY 2.0
#define BMP_PRESSURE_PAYLOAD_FLAG_OFFSET 0
#define BMP_PRESSURE_PAYLOAD_FLAG_BIT 3

// ******************************
// SCD40 CO2/temp/humidity sensor
// ******************************

DFRobot_SCD4X scd4x(&i2c, SCD4X_I2C_ADDR);
boolean scd4xStatus;
float scd4xTempValue;
unsigned int scd4xCO2Value;
float scd4xHumidityValue;

#define SCD_TEMP_MIN_VALUE -74.0
#define SCD_TEMP_MAX_VALUE 54.0
#define SCD_TEMP_ACCURACY 0.5
#define SCD_TEMP_PAYLOAD_FLAG_OFFSET 0
#define SCD_TEMP_PAYLOAD_FLAG_BIT 2
#define SCD_HUMIDITY_MIN_VALUE 0.0
#define SCD_HUMIDITY_MAX_VALUE 100.0
#define SCD_HUMIDITY_ACCURACY 1.0
#define SCD_HUMIDITY_PAYLOAD_FLAG_OFFSET 0
#define SCD_HUMIDITY_PAYLOAD_FLAG_BIT 1
#define SCD_CO2_MIN_VALUE 0.0
#define SCD_CO2_MAX_VALUE 2550.0
#define SCD_CO2_ACCURACY 10.0
#define SCD_CO2_PAYLOAD_FLAG_OFFSET 0
#define SCD_CO2_PAYLOAD_FLAG_BIT 0

// ***************
// Battery voltage
// ***************

float vBat;

#define VBAT_MIN_VALUE 0.0
#define VBAT_MAX_VALUE 4.5
#define VBAT_ACCURACY 0.02
#define VBAT_PAYLOAD_FLAG_OFFSET 1
#define VBAT_PAYLOAD_FLAG_BIT 7

// **********
// GPS module
// **********

#define GPS_SERIAL Serial2  // serial port
#define GPS_BAUD_RATE 9600  // baudrate
#define GPS_TIMEOUT 1500    // timeout in milliseconds

#define GPS_FAIL -1   // timeout error
#define GPS_NO_FIX 0  // no fix
#define GPS_FIX 1     // fix;

#define GPS_PAYLOAD_FLAG_OFFSET 1
#define GPS_PAYLOAD_FLAG_BIT 6

#define GPS_ALTITUDE_MIN_VALUE 0.0
#define GPS_ALTITUDE_MAX_VALUE 50000.0
#define GPS_ALTITUDE_ACCURACY 1.0

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

#define UBX_DELAY 100

int airborneMode = 0;
#define AIRBORNE_MODE 6

String airborneString = "(?)";

TinyGPS gps;

unsigned short oldSentences = 0;
float latitude;
float longitude;
float altitude;
float course;
float speed;
unsigned long date;
unsigned long hms;
unsigned long age;
unsigned char sats;
unsigned long fixAge;
boolean hasFix;

// **************
// EEPROM storage
// **************
#define EEPROM_SIZE 4          // Only 4 bytes are used
#define EEPROM_SEQNOUP_ADDR 0  // EEPROM address for SEQNOUP field (LMIC-related)

// ***********
// LoRa module
// ***********

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = { 26, 33, 32 }
};

#define LORAWAN_LOOP_DELAY 10     // Delay between 2 loops (in seconds)
#define LORAWAN_TX_LOOP_COUNT 8  // Loop count between 2 transmissions
#define LORAWAN_TX_ADR false      // Automatic Data Rate (disabled)
#define LORAWAN_TX_SF SF9         // Spread Factor (9)
#define LORAWAN_TX_POWER 14       // Tx Power (14dB, max)

#define LORAWAN_RESPONSE_BUFFER_SIZE 30
#define LORAWAN_APP_PAYLOAD_BUFFER_SIZE 51
int payloadSize = 0;

unsigned char LoRaWanResponse[LORAWAN_RESPONSE_BUFFER_SIZE];
int loopCount = -1;
unsigned char payload[LORAWAN_APP_PAYLOAD_BUFFER_SIZE];
static osjob_t sendjob;

// LMIC functions
void os_getDevEui(unsigned char *buf) {}
void os_getArtEui(unsigned char *buf) {}
void os_getDevKey(unsigned char *buf) {}

// ***********
// OLED module
// ***********

#define OLED_ADDR 0x3c  // I2C address
#define OLED_X 128      // Screen width in pixels
#define OLED_Y 64       // Screen height in pixels
#define OLED_LINE0 0    // Screen line 0 Y
#define OLED_LINE1 9    // Screen line 1 Y
#define OLED_LINE2 18   // Screen line 2 Y
#define OLED_LINE3 27   // Screen line 3 Y
#define OLED_LINE4 36   // Screen line 4 Y
#define OLED_LINE5 45   // Screen line 5 Y
#define OLED_LINE6 54   // Screen line 6 Y
Adafruit_SSD1306 oled(OLED_X, OLED_Y, &i2c, OLED_RST_PIN);
boolean oledStatus;

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

// *********
// SD module
// *********

#define SD_FILE_PREFIX "/data"  // absolute path to file (without prefix)
#define SD_FILE_SUFFIX ".txt"   // file suffix

SPIClass sdSPI(HSPI);
String sdFilePath = String(SD_FILE_PREFIX) + String(SD_FILE_SUFFIX);
boolean sdStatus;

// ------------------------------------
// Write a line of text to a file on SD
// ------------------------------------
// fs: filesystem reference
// path: path to file
// message: message to write
// mode: write mode (FILE_WRITE / FILE_APPEND)
// return true if message could be written to file
// -------------------------------------
boolean sdWriteFile(fs::FS &fs, const char *path, const char *message, const char *mode) {

  File file = fs.open(path, mode);

  if (!file)
    return false;
  boolean result = file.print(message);
  file.close();
  return result;
}

// *******************
// Camera Tilt (servo)
// *******************

#define CAMERA_TILT_SKY_ANGLE 0
#define CAMERA_TILT_HORIZON_ANGLE 85
#define CAMERA_TILT_GROUND_ANGLE 170

#define CAMERA_TILT_SERVO_MIN_PULSE_WIDTH 500
#define CAMERA_TILT_SERVO_MAX_PULSE_WIDTH 2450

#define CAMERA_STATE_RESET -1
#define CAMERA_STATE_TAKING_OFF 0
#define CAMERA_STATE_ASCENDING 1
#define CAMERA_STATE_AROUND_BURST 2
#define CAMERA_STATE_LANDING 3

#define CAMERA_STATE_TAKING_OFF_ALTITUDE_THRESHOLD 5000
#define CAMERA_STATE_BURST_ALTITUDE_THRESHOLD 20000
#define CAMERA_STATE_LANDING_ALTITUDE_THRESHOLD 5000

Servo cameraTilt; 
int cameraState;
int cameraAngleIndex;
#define CAMERA_ANGLE_COUNT 3
const int CAMERA_ANGLES[] = {CAMERA_TILT_SKY_ANGLE, CAMERA_TILT_HORIZON_ANGLE, CAMERA_TILT_GROUND_ANGLE };


// ###############
// Setup functions
// ###############

// ----------
// Set up all
// ----------

void setup() {

  EEPROM.begin(EEPROM_SIZE);

  #ifdef SERIAL_DEBUG_ON
    setupSerialDebug();
  #endif
  setupI2c();
  setupOled();

  setupGPS();
  switchGPSToAirborneMode();
  setupDHT();
  setupVBat();
//  setupI2c();
//  setupOled();
  setupBMP();
  setupSCD4x();
  setupSD();
  setupLMIC();
  setupCameraTilt();
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
  delay(1000);
  pinMode(GPS_ON_PIN, OUTPUT);   // v0.1
  digitalWrite(GPS_ON_PIN, HIGH);  // v0.1
  // pinMode(GPS_ON_PIN, OUTPUT_OPEN_DRAIN);  // v1.0
  // digitalWrite(GPS_ON_PIN, LOW);   // v1.0
  delay(500);
}

// -----------------
// Set up DHT sensor
// -----------------
void setupDHT() { dht.begin(); }

// ------------------
// Set up VBat sensor
// ------------------
void setupVBat() { pinMode(VBAT_ADC_PIN, INPUT); }

// --------------
// Set up I2C bus
// --------------
void setupI2c() {

    i2cStatus = i2c.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    #ifdef SERIAL_DEBUG_ON
      if (i2cStatus)
        Serial.println("I2C bus init ok");
      else
      Serial.println("I2C bus init failed");
    #endif
}

// ------------------
// Set up OLED module
// ------------------
void setupOled() {

  oledStatus = i2cStatus;    
  

  #ifdef SERIAL_DEBUG_ON
  if (oledStatus)
    Serial.println("Oled init ok");
  else
    Serial.println("Oled init failed!");
  #endif

  if (!oledStatus) return;
    
  pinMode(OLED_RST, OUTPUT);
  //pinMode(RST_OLED, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  //digitalWrite(RST_OLED, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  //digitalWrite(RST_OLED, HIGH);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);

  oledUpdateLine(OLED_LINE0, TRACKER_NAME, false);
}

// -----------------
// Set up BMP sensor
// -----------------
void setupBMP() {

  bmpStatus = i2cStatus;    
  if (!bmpStatus) return;

  bmpStatus = bmp.begin(BMP085_ULTRAHIGHRES, &i2c);

  #ifdef SERIAL_DEBUG_ON
    if (bmpStatus)
     Serial.println("BMP sensor found!");
    else
     Serial.println("BMP sensor not found!");
  #endif
}

// -------------------
// Set up SCD4x sensor
// -------------------
void setupSCD4x() {
  
  scd4xStatus = i2cStatus;    
  if (!scd4xStatus) return;

  scd4xStatus = scd4x.begin();

  #ifdef SERIAL_DEBUG_ON
    if (scd4xStatus)
     Serial.println("SCD4x sensor found!");
    else
     Serial.println("SCD4x sensor not found!");
  #endif

  if (scd4xStatus) {
    scd4x.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
    scd4x.setTempComp(4.0);   
    scd4x.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);   
  }
}

// --------------
// Set up LoRaWan
// --------------
void setupLMIC() {

  os_init();

  LMIC_reset();  // Reset MAC state
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Set up the channels to the defaults of most gateways
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);  // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);    // g2-band

  LMIC_setSession(0x1, DEV_ADDR, NWK_SKEY, APP_SKEY);

  LMIC_setLinkCheckMode(0);  // disable link check validation

  LMIC_setAdrMode(LORAWAN_TX_ADR);  // disable ADR

  LMIC.dn2Dr = DR_SF9;  // set SF9 for RX2 window (TTN recommended)

  LMIC_setDrTxpow(LORAWAN_TX_SF, LORAWAN_TX_POWER);  // set data rate and transmit power for uplink

  LMIC.seqnoUp = readLmicSeqNoUp();  // restore frame counter from EEPROM

  #ifdef SERIAL_DEBUG_ON
    Serial.println("frame counter (at reset) #" + String(LMIC.seqnoUp));
  #endif

  doSend(&sendjob);  // start job, will also fire up ABP join
}

// ---------
// Set up SD
// ---------
void setupSD() {

  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  sdStatus = SD.begin(SD_CS_PIN, sdSPI);
  
  #ifdef SERIAL_DEBUG_ON
    if (sdStatus)
      Serial.println("SD card found!");
    else
      Serial.println("SD card not found!");
  #endif

  if (!sdStatus) return;

  unsigned char cardType = SD.cardType();
  #ifdef SERIAL_DEBUG_ON
    Serial.print("SD type: ");
    if (cardType == CARD_MMC)
      Serial.println("MMC");
    else if (cardType == CARD_SD)
      Serial.println("SDSC");
    else if (cardType == CARD_SDHC)
      Serial.println("SDHC");
    else
      Serial.println("Unknown");
  #endif

  unsigned long cardSize = SD.cardSize() / (1024 * 1024);

  #ifdef SERIAL_DEBUG_ON
    Serial.printf("SD size : %lluMB\n", cardSize);
  #endif

  sdFilePath = String(SD_FILE_PREFIX) + "_" + String(EEPROM.read(EEPROM_SEQNOUP_ADDR)) + String(SD_FILE_SUFFIX);
}

// -------------------
// Sets up camera tilt
// -------------------
void setupCameraTilt() {

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	cameraTilt.setPeriodHertz(50);    // standard 50 hz servo
	cameraTilt.attach(CAMERA_TILT_PIN, CAMERA_TILT_SERVO_MIN_PULSE_WIDTH, CAMERA_TILT_SERVO_MAX_PULSE_WIDTH);
  cameraState = CAMERA_STATE_RESET;
  cameraAngleIndex = 0;
  updateCameraTilt();
}

// ---------
// main loop
// ---------
void loop() {
  os_runloop_once();
}

// --------
// send job
// --------
void doSend(osjob_t *j) {

  loopCount = (loopCount + 1) % LORAWAN_TX_LOOP_COUNT;

  #ifdef SERIAL_DEBUG_ON
    Serial.println("Loop " + String(loopCount));
  #endif
  
  trackerLoop();

  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(LORAWAN_LOOP_DELAY), doSend);
  if (loopCount > 0) return;

  updateCameraTilt();

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {

    #ifdef SERIAL_DEBUG_ON
      Serial.println(F("OP_TXRXPEND, not sending"));
    #endif
  }
  else {

    #ifdef RADIO_ON
      LMIC_setTxData2(1, payload, payloadSize, 0);  // Prepare upstream data transmission
    #endif

    saveLmicSeqNoUp();

    #ifdef SERIAL_DEBUG_ON
      Serial.println("Sending uplink packet..." + String(LMIC.seqnoUp));
    #endif

    oledUpdateLine(OLED_LINE6, String(LMIC.seqnoUp) + " ->", true);
  }
}

// -----------------
// On event callback
// -----------------
void onEvent(ev_t ev) {

  #ifdef SERIAL_DEBUG_ON
    Serial.print(os_getTime());
    Serial.print(": ");
  #endif
  switch (ev) {
    case EV_TXCOMPLETE:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      #endif

      oledUpdateLine(OLED_LINE6, String(LMIC.seqnoUp) + " !", true);

      if (LMIC.txrxFlags & TXRX_ACK)

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("Received ack"));
      #endif

      if (LMIC.dataLen) {  // Data received in rx slot after tx

        #ifdef SERIAL_DEBUG_ON
          Serial.print(F("Data Received: "));
          Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
          Serial.println();
          Serial.println(LMIC.rssi);
        #endif
      }
      break;
    
    case EV_JOINING:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("EV_JOINING: -> Joining..."));
      #endif
      break;
    case EV_JOINED:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("EV_JOINED"));
      #endif

      LMIC_setLinkCheckMode(0);  // Disable link check validation (automatically enabled during join, but not supported by TTN at this time)
      break;
    
    case EV_RXCOMPLETE:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("EV_RXCOMPLETE"));
      #endif
      break;
    
    case EV_LINK_DEAD:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("EV_LINK_DEAD"));
      #endif
      break;
    
    case EV_LINK_ALIVE:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("EV_LINK_ALIVE"));
      #endif
      break;
    
    default:

      #ifdef SERIAL_DEBUG_ON
        Serial.println(F("Unknown event"));
      #endif    
  }
}

// --------
// App loop
// --------
void trackerLoop() {

  updateSensors();
  updatePayload();
  dataOledDisplay();

  #ifdef SERIAL_DEBUG_ON
    dataSerialPrint();
  #endif
  
  dataSdWrite();
}

// #############
// App functions
// #############

// ------------------
// Update sensor data
// ------------------
void updateSensors() {

  updateDHT();
  updateBMP();
  updateSCD4x();
  updateVBat();
  updateGPS();
}

// ----------------------
// Update DHT sensor data
// ----------------------
void updateDHT() {

  dhtTempValue = dht.readTemperature();
  dhtHumidityValue = dht.readHumidity();
}

// ----------------------
// Update BMP sensor data
// ----------------------
void updateBMP() {

  if (!bmpStatus) return;

  bmpTempValue = bmp.readTemperature();
  bmpPressureValue = bmp.readPressure();
}

// ------------------------
// Update SCD4x sensor data
// ------------------------
void updateSCD4x() {

  if (!scd4xStatus) return;

  DFRobot_SCD4X::sSensorMeasurement_t data;
  scd4x.readMeasurement(&data);

  scd4xTempValue = data.temp;
  scd4xCO2Value = data.CO2ppm;
  scd4xHumidityValue = data.humidity;
}

// -----------------------
// Update VBat sensor data
// -----------------------
void updateVBat() { vBat = analogRead(VBAT_ADC_PIN) * 3.3 / 4096.0 / VBAT_DIVIDER; }

// ---------------
// Update GPS data
// ---------------
// return:
//  - GPS_FAIL, if a timeout occurs during sentence reading
//  - GPS_FIX, if there is a fix
//  - GPS_NO_FIX, if there is no fix
// ----------------
int updateGPS() {

  int result;
  boolean newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  for (unsigned long start = millis(); millis() - start < GPS_TIMEOUT;) {
    while (GPS_SERIAL.available()) {
      char c = GPS_SERIAL.read();
      #ifdef SERIAL_DEBUG_ON
        Serial.write(c);
      #endif
      if (gps.encode(c))
        newData = true;
    }
  }

  if (newData) {
    gps.get_datetime(&date, &hms, &age);
    gps.f_get_position(&latitude, &longitude, &fixAge);
    hasFix = (fixAge > 0) && (fixAge < 5000);
    sats = gps.satellites();
    altitude = gps.f_altitude();
    course = gps.f_course();
    speed = gps.f_speed_kmph();
  }

  gps.stats(&chars, &sentences, &failed);
  if (chars == 0)
    result = GPS_FAIL;
  if (sentences == oldSentences)
    result = GPS_FAIL;

  oldSentences = sentences;

  if (hasFix)
    result = GPS_FIX;
  else
    result = GPS_NO_FIX;

  return result;
}

// --------------------------
// Update LoRaWAN app payload
// --------------------------
void updatePayload() {

  int offset = 2;
  resetSensorFlagsPayload();
  
  offset = updateSysTimePayload(offset);
  offset = updateDHTPayload(offset);
  offset = updateBMPPayload(offset);
  offset = updateSCDPayload(offset);
  offset = updateVBatPayload(offset);
  offset = updateGPSPayload(offset);
  payloadSize = offset;
}

// -----------------------------
// Update SysTime sensor payload
// -----------------------------
int updateSysTimePayload(int offset) {
  unsigned short uptime = (millis() / 1000);
  setSensorFlagPayload(SYSTIME_PAYLOAD_FLAG_OFFSET, SYSTIME_PAYLOAD_FLAG_BIT);
  payload[offset++] = ((uint8_t *)&uptime)[0];
  payload[offset++] = ((uint8_t *)&uptime)[1]; 
  return offset;
}

// -------------------------
// Update DHT sensor payload
// -------------------------
int updateDHTPayload(int offset) {
  if ((dhtTempValue >= DHT_TEMP_MIN_VALUE)&&(dhtTempValue <= DHT_TEMP_MAX_VALUE)) {
    setSensorFlagPayload(DHT_TEMP_PAYLOAD_FLAG_OFFSET, DHT_TEMP_PAYLOAD_FLAG_BIT);
    unsigned char dhtTempPayloadValue = (unsigned char)((dhtTempValue - DHT_TEMP_MIN_VALUE)/DHT_TEMP_ACCURACY);
    payload[offset++] = dhtTempPayloadValue;
  }
  if ((dhtHumidityValue >= DHT_HUMIDITY_MIN_VALUE)&&(dhtHumidityValue <= DHT_HUMIDITY_MAX_VALUE)) {
    setSensorFlagPayload(DHT_HUMIDITY_PAYLOAD_FLAG_OFFSET, DHT_HUMIDITY_PAYLOAD_FLAG_BIT);
    unsigned char dhtHumidityPayloadValue = (unsigned char)((dhtHumidityValue - DHT_HUMIDITY_MIN_VALUE)/DHT_HUMIDITY_ACCURACY);
    payload[offset++] = dhtHumidityPayloadValue;
  }
  return offset;
}

// -------------------------
// Update BMP sensor payload
// -------------------------
int updateBMPPayload(int offset) {
  if (!bmpStatus) 
    return offset;
  
  if ((bmpTempValue >= BMP_TEMP_MIN_VALUE)&&(bmpTempValue <= BMP_TEMP_MAX_VALUE)) {
    setSensorFlagPayload(BMP_TEMP_PAYLOAD_FLAG_OFFSET, BMP_TEMP_PAYLOAD_FLAG_BIT);
    unsigned char bmpTempPayloadValue = (unsigned char)((bmpTempValue - BMP_TEMP_MIN_VALUE)/BMP_TEMP_ACCURACY);
    payload[offset++] = bmpTempPayloadValue;
  }

  if ((bmpPressureValue >= BMP_PRESSURE_MIN_VALUE)&&(bmpPressureValue <= BMP_PRESSURE_MAX_VALUE)) {
    setSensorFlagPayload(BMP_PRESSURE_PAYLOAD_FLAG_OFFSET,BMP_PRESSURE_PAYLOAD_FLAG_BIT);
    unsigned short bmpPressurePayloadValue = (unsigned short)((bmpPressureValue - BMP_PRESSURE_MIN_VALUE)/BMP_PRESSURE_ACCURACY);
    payload[offset++] = ((uint8_t *)&bmpPressurePayloadValue)[0];
    payload[offset++] = ((uint8_t *)&bmpPressurePayloadValue)[1];
  }

  return offset;
}

// -------------------------
// Update SCD sensor payload
// -------------------------
int updateSCDPayload(int offset) {
  if (!i2cStatus) 
    return offset;
  
  boolean falseData = (scd4xTempValue == -45.0)&&(scd4xHumidityValue == 0.0)&&(scd4xCO2Value == 0.0);
  if (falseData) 
    return offset;

  if ((scd4xTempValue >= SCD_TEMP_MIN_VALUE)&&(scd4xTempValue <= SCD_TEMP_MAX_VALUE)) {
    setSensorFlagPayload(SCD_TEMP_PAYLOAD_FLAG_OFFSET, SCD_TEMP_PAYLOAD_FLAG_BIT);
    unsigned char scd4xTempPayloadValue = (unsigned char)((scd4xTempValue - SCD_TEMP_MIN_VALUE)/SCD_TEMP_ACCURACY);
    payload[offset++] = scd4xTempPayloadValue;
  }

  if ((scd4xHumidityValue >= SCD_HUMIDITY_MIN_VALUE)&&(scd4xHumidityValue <= SCD_HUMIDITY_MAX_VALUE)) {
    setSensorFlagPayload(SCD_HUMIDITY_PAYLOAD_FLAG_OFFSET, SCD_HUMIDITY_PAYLOAD_FLAG_BIT);
    unsigned char scd4xHumidityPayloadValue = (unsigned char)((scd4xHumidityValue - SCD_HUMIDITY_MIN_VALUE)/SCD_HUMIDITY_ACCURACY);
    payload[offset++] = scd4xHumidityPayloadValue;
  }

  if ((scd4xCO2Value >= SCD_CO2_MIN_VALUE)&&(scd4xCO2Value <= SCD_CO2_MAX_VALUE)) {
    setSensorFlagPayload(SCD_CO2_PAYLOAD_FLAG_OFFSET, SCD_CO2_PAYLOAD_FLAG_BIT);
    unsigned char scd4xC02PayloadValue = (unsigned char)((scd4xCO2Value - SCD_CO2_MIN_VALUE)/SCD_CO2_ACCURACY);
    payload[offset++] = scd4xC02PayloadValue;
  }

  return offset;
}

// --------------------------
// Update VBat sensor payload
// --------------------------
int updateVBatPayload(int offset) {
  if ((vBat >= VBAT_MIN_VALUE)&&(vBat <= VBAT_MAX_VALUE)) {
    setSensorFlagPayload(VBAT_PAYLOAD_FLAG_OFFSET, VBAT_PAYLOAD_FLAG_BIT);
    unsigned char vBatPayloadValue = (unsigned char)((vBat - VBAT_MIN_VALUE)/VBAT_ACCURACY);
    payload[offset++] = vBatPayloadValue;
  }

  return offset;
}

// ------------------
// Update GPS payload
// ------------------
int updateGPSPayload(int offset) {
  if (hasFix) {
    setSensorFlagPayload(GPS_PAYLOAD_FLAG_OFFSET, GPS_PAYLOAD_FLAG_BIT);
    payload[offset++] = ((uint8_t *)&hms)[0];
    payload[offset++] = ((uint8_t *)&hms)[1];
    payload[offset++] = ((uint8_t *)&hms)[2];
    payload[offset++] = ((uint8_t *)&hms)[3];
    unsigned short altitudePayloadValue = (unsigned short)((altitude - GPS_ALTITUDE_MIN_VALUE)/GPS_ALTITUDE_ACCURACY);
    payload[offset++] = ((uint8_t *)&altitudePayloadValue)[0];
    payload[offset++] = ((uint8_t *)&altitudePayloadValue)[1]; 
    payload[offset++] = ((uint8_t *)&latitude)[0];
    payload[offset++] = ((uint8_t *)&latitude)[1];
    payload[offset++] = ((uint8_t *)&latitude)[2];
    payload[offset++] = ((uint8_t *)&latitude)[3];
    payload[offset++] = ((uint8_t *)&longitude)[0];
    payload[offset++] = ((uint8_t *)&longitude)[1];
    payload[offset++] = ((uint8_t *)&longitude)[2];
    payload[offset++] = ((uint8_t *)&longitude)[3];
    payload[offset++] = ((uint8_t ) course);
    unsigned char halfSpeedValue = (unsigned char) (speed / 2.0);
    payload[offset++] = halfSpeedValue;
  }

  return offset;
}

// ------------------
// Reset sensor flags
// ------------------
void resetSensorFlagsPayload() {
  payload[0] = 0;
  payload[1] = 0;
}

// -----------------
// Unset sensor flag
// -----------------
void unsetSensorFlagPayload(int flagOffset, int flagBit) {
  payload[flagOffset] = payload[flagOffset] & ((0b11111111) ^ (0b00000001 << (flagBit)));
}

// -----------------
// Set sensor flag
// -----------------
void setSensorFlagPayload(int flagOffset, int flagBit) {
  payload[flagOffset] = payload[flagOffset] | (0b00000001 << (flagBit));
}

// ------------------
// Update camera tilt
// ------------------
void updateCameraTilt() {
  updateCameraState();
  
  switch (cameraState) {
    case CAMERA_STATE_RESET:
      for (int index=0; index < 3; index++) {
         cameraTilt.write(CAMERA_ANGLES[index]);
         delay(1000);
      }
      cameraState = CAMERA_STATE_TAKING_OFF;
      return;
    case CAMERA_STATE_TAKING_OFF:
    case CAMERA_STATE_LANDING:
      cameraTilt.write(CAMERA_TILT_GROUND_ANGLE);    // tell servo to go to position in variable 'pos'
		  return;
    case CAMERA_STATE_ASCENDING:
      cameraAngleIndex = (cameraAngleIndex + 1) % 3;
      cameraTilt.write(CAMERA_ANGLES[cameraAngleIndex]);
      return;
    case CAMERA_STATE_AROUND_BURST:
    default:
      cameraTilt.write(CAMERA_TILT_SKY_ANGLE);
      return;
    }
}


// -------------------
// Update camera state
// -------------------
void updateCameraState() {
  switch (cameraState) {
    case CAMERA_STATE_TAKING_OFF:
      if (hasFix && altitude > CAMERA_STATE_TAKING_OFF_ALTITUDE_THRESHOLD)
        cameraState = CAMERA_STATE_ASCENDING;
      return;
    case CAMERA_STATE_ASCENDING:
      if (hasFix && altitude > CAMERA_STATE_BURST_ALTITUDE_THRESHOLD)
        cameraState = CAMERA_STATE_AROUND_BURST;
      return;
    case CAMERA_STATE_AROUND_BURST:
      if (hasFix && altitude < CAMERA_STATE_LANDING_ALTITUDE_THRESHOLD)
        cameraState = CAMERA_STATE_LANDING;
      return;  
    }
 }

// --------------------
// Display data on OLED
// --------------------
void dataOledDisplay() {

  if (!oledStatus) return;

  String line0 = String(TRACKER_NAME) + " " + airborneString;
  String line1 = "DHT " + String(dhtTempValue) + "C / " + String(dhtHumidityValue) + "%";
  String line2 = "BMP " + String(bmpTempValue) + "C / " + String(bmpPressureValue) + "Pa";

  String line3 = "SCD " + String(scd4xTempValue) + "C / " + String(scd4xHumidityValue) + "%";
  String line4 = "SCD " + String(scd4xCO2Value) + "ppm / ";
  String line5 = "Fix / ";
  if (!hasFix)
    line5 = "no " + line5;
  line5 += " " + String(vBat) + "V ";
  String line6 =   String(LMIC.seqnoUp);

  oled.clearDisplay();
  oledUpdateLine(OLED_LINE0, line0, false);
  oledUpdateLine(OLED_LINE1, line1, false);
  oledUpdateLine(OLED_LINE2, line2, false);
  oledUpdateLine(OLED_LINE3, line3, false);
  oledUpdateLine(OLED_LINE4, line4, false);
  oledUpdateLine(OLED_LINE5, line5, false);
  oledUpdateLine(OLED_LINE6, line6, false);
}

// --------------------------
// Print data on serial debug
// --------------------------
void dataSerialPrint() {
  
  Serial.print("DHT Temp: ");
  Serial.print(dhtTempValue);
  Serial.print("DHT Hum: ");
  Serial.print(dhtHumidityValue);
  Serial.print("BMP Temp: ");
  Serial.print(bmpTempValue);
  Serial.println(" °C");
  Serial.print("BMP Pressure: ");
  Serial.print(bmpPressureValue);
  Serial.println(" Pa");
  Serial.print("SCD Temp: ");
  Serial.print(scd4xTempValue);
  Serial.print("SCD Hum: ");
  Serial.print(scd4xHumidityValue);
  Serial.print("SCD CO2: ");
  Serial.print(scd4xCO2Value);
  Serial.print("VBat: ");
  Serial.print(vBat);
  Serial.println(" V");
  if (hasFix) {
    Serial.println("GPS Fix");
    Serial.print("Lat: ");
    Serial.println(String(latitude, 5));
    Serial.print("Long: ");
    Serial.println(String(longitude, 5));
    Serial.print("altitude: ");
    Serial.println(String(altitude, 0));
  } 
  else
    Serial.println("No GPS Fix");

  Serial.print("sats: ");
  Serial.println(sats);
  Serial.print("time: ");
  Serial.println(hms);
  Serial.print("course: ");
  Serial.println(course);
  Serial.print("speed: ");
  Serial.println(speed);
  Serial.print("cameraState: ");
  Serial.println(cameraState);
}

// ----------------
// Write data on SD
// ----------------
void dataSdWrite() {

  // TBC scd4x
  // 416,117,3,22.00,29.10,22.10,99917,7.58,1,11590200,44.92,4.92,142.90,4,104.94,1.74
  String dataStr = String(millis() / 1000) + "," + String(LMIC.seqnoUp) + "," + String(loopCount) + ",";
  dataStr += String(dhtTempValue) + "," + String(dhtHumidityValue) + ",";
  dataStr += String(bmpTempValue) + "," + String(bmpPressureValue) + ",";
  dataStr += String(scd4xTempValue) + "," + String(scd4xHumidityValue) + "," + String(scd4xCO2Value) + ",";
  dataStr += String(vBat) + ",";
  if (hasFix)
    dataStr += "1,"
  else
    dataStr += "0,";
  dataStr += String(hms) + "," + String(latitude, 5) + "," + String(longitude, 5) + "," + String(altitude, 1) + "," + String(sats) + "," + String(course) + "," + String(speed, 1) + "," ;
  dataStr += String(cameraState);
  dataStr += "\r\n";

  boolean result;
  File file = SD.open(sdFilePath);
  if (!file) {

    #ifdef SERIAL_DEBUG_ON
      Serial.println("SD file creation");
    #endif
    
    result = sdWriteFile(SD, sdFilePath.c_str(), dataStr.c_str(), FILE_WRITE);
  } 
  else
    result = sdWriteFile(SD, sdFilePath.c_str(), dataStr.c_str(), FILE_APPEND);

  #ifdef SERIAL_DEBUG_ON
    if (result)
      Serial.println("Data writing successful");
    else
      Serial.println("Data writing failed");
  #endif

  file.close();
}

void switchGPSToAirborneMode() {
  oledUpdateLine(OLED_LINE1, "muting NMEA", true);
  Serial.println("muting nmea");
  gpsSentencesMute();
  delay(2000);

  oledUpdateLine(OLED_LINE1, "reading AirBorne", true);  
  airborneMode = getGPSAirborne();
  airborneString = "";
  if (airborneMode != AIRBORNE_MODE) 
    airborneString = "(X)";
  else
     airborneString = "(" + String(airborneMode) + ")";
  oledUpdateLine(OLED_LINE2, airborneString, true);  
  delay(UBX_DELAY);

  oledUpdateLine(OLED_LINE1, "switching airBorne", true);  
  sendGPSCommand(SET_AIRBORNE1G, 44);
  delay(UBX_DELAY);
  
  oledUpdateLine(OLED_LINE1, "reading AirBorne", true);  
  airborneMode = getGPSAirborne();
  
  if (airborneMode != AIRBORNE_MODE) 
    airborneString = "(X)";
  else
     airborneString = "(" + String(airborneMode) + ")";
  oledUpdateLine(OLED_LINE2, airborneString, true);  
  // delay(UBX_DELAY);

  oledUpdateLine(OLED_LINE1, "unmuting NMEA", true);
  gpsSentencesUnmute();
  oledUpdateLine(OLED_LINE1, "", true);
  //delay(UBX_DELAY);
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
    if (index > 8) return UBX_RESPONSE[8];
  }
  
  return 0;
}

void saveLmicSeqNoUp() {
  writeUnsignedIntIntoEEPROM(EEPROM_SEQNOUP_ADDR, LMIC.seqnoUp);

}

int readLmicSeqNoUp() {
  return readUnsignedIntFromEEPROM(EEPROM_SEQNOUP_ADDR);  
}

int readUnsignedIntFromEEPROM(int address)
{
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (unsigned int) ((byte1 << 8) + byte2);
}

void writeUnsignedIntIntoEEPROM(int address, int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
  EEPROM.commit();
}


