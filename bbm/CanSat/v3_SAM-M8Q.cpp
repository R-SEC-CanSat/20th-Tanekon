
#include <MicroNMEA.h>
#include <Wire.h>
#include <SPI.h>

//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x42
#define I2C_DELAY 1

#define RESET_PIN 10


// Refer to Stream devices by use
TwoWire& gps = Wire;

//I2C read data structures
char buff[32];
int idx = 0;

//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

volatile bool ppsTriggered = false;

void ppsHandler(void)
{
  ppsTriggered = true;
}

void gpsHardwareReset()
{
  //reset the device
  digitalWrite(RESET_PIN, LOW);
  delay(50);
  digitalWrite(RESET_PIN, HIGH);

  //wait for reset to apply
  delay(2000);

}
//Read 32 bytes from I2C
void readI2C(char *inBuff)
{/*
  gps.beginTransmission(DEFAULT_DEVICE_ADDRESS);
  gps.write((uint8_t) DEFAULT_DEVICE_PORT);
  gps.endTransmission(false);
  */
  gps.requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) 32);
  int i = 0;
  while (gps.available()) {
    inBuff[i] = gps.read();
    i++;
  }
}


void setup(void)
{
  Serial.begin(115200); // Serial
  gps.begin(); // gps

  //Start the module
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  gpsHardwareReset();

  //Reinitialize I2C after the reset
  gps.begin();

  //clear i2c buffer
  char c;
  idx = 0;
  memset(buff, 0, 32);
  do {
    if (idx == 0) {
      readI2C(buff);
      delay(I2C_DELAY);
    }
    c = buff[idx];
    idx++;
    idx %= 32;
  }
  while ((uint8_t) c != 0xFF);//-1
}

void loop(void)
{Serial.println("loop");
  //Read 32 bytes from I2C
  readI2C(buff);
  delay(I2C_DELAY);

  //Parse the NMEA message
  nmea.parse(buff, 32);

  //Print the NMEA message
  Serial.println(nmeaBuffer);
  delay(1000);
  
}