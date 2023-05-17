#include <EEPROM.h>

#define EEPROM_SIZE 512 
#define LINE_SIZE 32

void setup(){
    EEPROM.begin(EEPROM_SIZE);
    Serial.begin(115200);
    delay(5000);
    Serial.println("Dumping EEPROM");
    dumpEEPROM();
    Serial.println();
    Serial.println("Erasing EEPROM");
    eraseEEPROM();
    Serial.println();
    Serial.println("Dumping EEPROM");
    dumpEEPROM();
}

void loop() {}

void dumpEEPROM() {
  for (int address=0;address<EEPROM_SIZE;address++) {
    Serial.print(EEPROM.read(address), HEX);
    if ((address + 1) % LINE_SIZE == 0)
      Serial.println();
    else 
      Serial.print(" ");
  }
}

void eraseEEPROM() {
  for (int address = 0; address < EEPROM_SIZE; address++) {
    EEPROM.write(address, 0);
  }
  EEPROM.commit();
  delay(500);
}