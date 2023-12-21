

  /*
  Read NMEA sentences over I2C using u-blox module SAM-M8Q, NEO-M8P, ZED-F9P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA setences from the u-blox module over I2c and outputs
  them to the serial port
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Hardware connections:
  BME280 -> Arduino
  GND -> GND
  3.3 -> 3.3
  CS -> 2
  MOSI -> 11
  MISO -> 12
  SCK -> 13


*/

#include <Wire.h> //Needed for I2C to GNSS
#include <M5Core2.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);//GGA以外は通さない
  myGNSS.setHighPrecisionMode(true);//高精度モードらしい
  //This will pipe all NMEA sentences to the serial port so we can see them
  myGNSS.setNMEAOutputPort(Serial);
}

void loop()
{ 
    Serial.println(GPSdata()[1]);

  delay(250); //Don't pound too hard on the I2C bus
}
(long, long) GPSdata(){
  myGNSS.checkUblox();//初期化

  long latitude_mdeg = nmea.getLatitude();//小数点以下6桁を小数点無しで出す
  long longitude_mdeg = nmea.getLongitude();

  return (latitude_mdeg,longitude_mdeg);
}
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}
