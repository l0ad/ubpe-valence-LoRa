
#include <DFRobot_SCD4X.h>

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 15

TwoWire i2c = TwoWire(1);
DFRobot_SCD4X SCD4X(&i2c, SCD4X_I2C_ADDR);

void setup(void)
{
  i2c.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  Serial.begin(115200);

  while( !SCD4X.begin() ){
    Serial.println("Device not found");
    delay(3000);
  }
 
  SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);

  SCD4X.setTempComp(4.0);

  Serial.println("Calibration value (in ppm, or 0 to ignore forced calibration)? ");
  while (Serial.available() == 0) {} 
  int calibrationValue = Serial.readString().toInt(); 

  if (calibrationValue != 0) {

    Serial.print("Forced calibration ");
    if (SCD4X.performForcedRecalibration(calibrationValue) == 0x7fff)
      Serial.println("failed");
    else
      Serial.println("success");
  }

  Serial.println("Enable auto calibration (y/n)? ");
  while (Serial.available() == 0) {} 
  String line = Serial.readString();
  line.toUpperCase();
  line.trim();
  SCD4X.setAutoCalibMode(line == "Y");
  Serial.print("Auto calibration ");
  if(SCD4X.getAutoCalibMode())
    Serial.println("enabled");
  else 
    Serial.println("disabled");

  Serial.println("Write settings (y/n)? ");
  while (Serial.available() == 0) {} 
  line = Serial.readString();
  line.toUpperCase();
  line.trim();
  if (line == "Y") {
    Serial.println("Writing settings");
    SCD4X.persistSettings();
  }
  
  Serial.println("Reinitializing module");
  SCD4X.moduleReinit();

  Serial.println("Starting periodic measurement");
  SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);

  Serial.println();
}

void loop()
{

  if(SCD4X.getDataReadyStatus()) {   
    
    DFRobot_SCD4X::sSensorMeasurement_t data;
    SCD4X.readMeasurement(&data);

    Serial.print("CO2: ");
    Serial.print(data.CO2ppm);
    Serial.println(" ppm");

    Serial.print("Temperature:");
    Serial.print(data.temp);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.print(data.humidity);
    Serial.println(" %");

    Serial.println();
  }
  delay(5000);
}
