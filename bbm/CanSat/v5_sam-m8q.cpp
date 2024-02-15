
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


void ppsHandler(void);


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
{
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
  Serial.println("Resetting GPS module ...");
  gpsHardwareReset();
  Serial.println("... done");


  delay(1000);

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
  while ((uint8_t) c != 0xFF);

  pinMode(11, INPUT);
  attachInterrupt(digitalPinToInterrupt(11), ppsHandler, RISING);
}

void loop(void)
{
  //If a message is recieved print all the informations
  if (ppsTriggered) {
    ppsTriggered = false;


    Serial.print("Date/time: ");
    Serial.print(nmea.getYear());
    Serial.print('-');
    Serial.print(int(nmea.getMonth()));
    Serial.print('-');
    Serial.print(int(nmea.getDay()));
    Serial.print('T');
    Serial.print(int(nmea.getHour()));
    Serial.print(':');
    Serial.print(int(nmea.getMinute()));
    Serial.print(':');
    Serial.println(int(nmea.getSecond()));

    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);

    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);


    Serial.print("Speed: ");
    Serial.println(nmea.getSpeed() / 1000., 3);
    Serial.print("Course: ");
    Serial.println(nmea.getCourse() / 1000., 3);
    Serial.println("-----------------------");
    nmea.clear();
  }

  //While the message isn't complete
  while (!ppsTriggered) {
    char c ;
    if (idx == 0) {
      readI2C(buff);
      delay(I2C_DELAY);
    }
    //Fetch the character one by one
    c = buff[idx];
    idx++;
    idx %= 32;
    //If we have a valid character pass it to the library
    if ((uint8_t) c != 0xFF) {
      Serial.print(c);
      nmea.process(c);
    }
  }

}