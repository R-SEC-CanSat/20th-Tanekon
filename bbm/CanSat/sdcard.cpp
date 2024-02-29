#include <Arduino.h>
#include "SD.h"
#include <Wire.h>
/*//SPI V
#define SD_CS        21
#define SD_MOSI      23
#define SD_MISO      19
#define SD_SCK       18
*/
///SPI H

#define SD_MOSI      27
#define SD_MISO      25
#define SD_SCK       26

#define SD_CS_PIN 14

File myFile;
SPIClass SPISD(HSPI);
void SD_Init()
{
  //  SPIClass SPI2(HSPI);


    SPISD.begin(SD_SCK, SD_MISO, SD_MOSI);
    if (!SD.begin(SD_CS_PIN,SPISD)) {  //SD_CS_PIN this pin is just the dummy pin since the SD need the input 
    Serial.println(F("failed!"));
    return;
    }
    else Serial.println(F("SD read!"));
    myFile = SD.open("/test.txt", "a"); //append to file
  if (myFile)
  {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.close();
    Serial.println("done.");
  }
  else
  {
    Serial.println("error opening test.txt to write");
  }
  myFile = SD.open("/test.txt", "r"); //read from file
  if (myFile)
  {
    Serial.println("test.txt:");
    String inString;  //need to use Strings because of the ESP32 webserver
    while (myFile.available())
    {
      inString += myFile.readString();
    }
    myFile.close();
    Serial.print(inString);
  }
  else
  {
    Serial.println("error opening test.txt to read");
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SD_Init();
}
void loop() {
  // put your main code here, to run repeatedly:
}
 