#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "pinout.h"

SPIClass sdSPI(HSPI);

void setup(){
    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);    
    Serial.begin(115200);
    Serial.println();
    Serial.println("Starting...");
    Serial.println("Looking for SD card...");
    if(!SD.begin(SD_CS_PIN, sdSPI)){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD found!");
        return;
    }
    
    Serial.println("SD found!");
    Serial.print(SD.cardSize() / (1024 * 1024));
    Serial.println(" MB");
}

void loop(){ processCommand(readCommand()); }

String readCommand() {
  Serial.println();
  Serial.print("> ");
  Serial.flush();
  while (Serial.available() == 0) {} 
  String command = Serial.readString(); 
  command.trim();
  return command;
}

void processCommand(String command) {
  if (command.startsWith("HELP")) {   
    processHelp();
    return;
  } 
  if (command.startsWith("DIR")) {
    processDir();
    return;
  }
  if (command.startsWith("DUMP")) {
    command = command.substring(String("DUMP").length());
    command.trim();
    processDump(command);
    return;
  }
  if (command.startsWith("RENAME")) {
    command = command.substring(String("RENAME").length());
    command.trim();
    processRename(command);
    return;
  }
  if (command.startsWith("DEL")) {
    command = command.substring(String("DEL").length());
    command.trim();
    processDelete(command);
    return;
  } 
  if (command.startsWith("WIPE")) {    
    processWipe();
    return;
  }
  else Serial.println("Command not supported or syntax error. type 'HELP' for command list");
}

void processHelp() {
  Serial.println();
  Serial.println("List of available commands:");
  Serial.println("DIR \t\t list files in root directory");
  Serial.println("DUMP name \t dump content of a file");
  Serial.println("RENAME old new \t rename a file");
  Serial.println("DEL name \t delete a file");
  Serial.println("WIPE \t\t wipe all files");
} 

void processDir() {
  int count = 0;
  File dir = SD.open("/");
    if(!dir){
        Serial.println("Failed to open directory");
        return;
    }
  Serial.println();
  File file = dir.openNextFile();
  while(file){
      if(!file.isDirectory()){
          count ++;
          Serial.print(file.name());
          Serial.print(" (");
          Serial.print(file.size());
          Serial.println(" bytes)");
      }
      file = dir.openNextFile();
  }
  Serial.println(String(count) + " files");
}

void processDump(String filename){
   
    File file = SD.open("/" + filename);
    if(!file){
        Serial.println("Failed to open " + filename);
        return;
    }
    Serial.println();
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void processRename(String fileNames){  
    int spaceIndex = fileNames.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Syntax error!");
      return;
    }
    String oldFileName = fileNames.substring(0, spaceIndex);
    oldFileName.trim();
    String newFileName = fileNames.substring(spaceIndex+1);
    oldFileName.trim();

    Serial.println();
    if (SD.rename("/" + oldFileName, "/" + newFileName)) {
        Serial.println(oldFileName + " successfully renamed to " + newFileName);
    } 
    else {
        Serial.println("Error while renaming " + oldFileName + " to " + newFileName);
    }
}

void processDelete(String filename){   
    Serial.println(); 
    if(SD.remove("/" + filename)){
         Serial.println(filename + " successfully deleted");
    } 
    else {
        Serial.println("Error while deleting " + filename);
    }
}

void processWipe() {
  int count = 0;
  File dir = SD.open("/");
    if(!dir){
        Serial.println("Failed to open directory");
        return;
    }
  Serial.println();
  File file = dir.openNextFile();
  while(file){
      if(!file.isDirectory()) {
          Serial.print(".");
          if (SD.remove("/" + String(file.name())))
            count ++;
      }
      file = dir.openNextFile();      
  }
  Serial.println();
  Serial.println(String(count) + " files deleted");
}

