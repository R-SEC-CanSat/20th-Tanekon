
#include <MicroNMEA.h>
#include <Wire.h>

//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x42
#define DEFAULT_DEVICE_PORT 0xFF//拾った情報を書き込む場所
#define I2C_DELAY 1

#define RESET_PIN 7

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#endif

#ifdef ARDUINO_ARCH_STM32
#define DEV_I2C Wire
#endif

#ifdef ARDUINO_ARCH_AVR
#define DEV_I2C Wire
#endif

// Refer to Stream devices by use
HardwareSerial& console = Serial;
TwoWire& gps = DEV_I2C;

//I2C read data structures
char buff[32];
int idx = 0;

//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));


bool ledState = LOW;
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
  gps.beginTransmission(DEFAULT_DEVICE_ADDRESS);
  gps.write((uint8_t) DEFAULT_DEVICE_PORT);
  gps.endTransmission(false);
  gps.requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) 32);
  int i = 0;
  while (gps.available()) {
    inBuff[i] = gps.read();
    i++;
  }
}


void setup(void)
{
  console.begin(115200); // console
  gps.begin(); // gps

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);

  //Start the module
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  console.println("Resetting GPS module ...");
  gpsHardwareReset();
  console.println("... done");

  // Change the echoing messages to the ones recognized by the MicroNMEA library
  sendCommand("$PSTMSETPAR,1231,0x00000042");
  sendCommand("$PSTMSAVEPAR");

  //Reset the device so that the changes could take plaace
  sendCommand("$PSTMSRR");

  delay(4000);

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

  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), ppsHandler, RISING);
}

void loop(void)
{
  //If a message is recieved print all the informations
  if (ppsTriggered) {
    ppsTriggered = false;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);

    // Output GPS information from previous second
    console.print("Valid fix: ");
    console.println(nmea.isValid() ? "yes" : "no");

    console.print("Nav. system: ");
    if (nmea.getNavSystem())
      console.println(nmea.getNavSystem());
    else
      console.println("none");

    console.print("Num. satellites: ");
    console.println(nmea.getNumSatellites());

    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    console.print("Latitude (deg): ");
    console.println(latitude_mdeg / 1000000., 6);

    console.print("Longitude (deg): ");
    console.println(longitude_mdeg / 1000000., 6);

    console.print("Speed: ");
    console.println(nmea.getSpeed() / 1000., 3);
    console.print("Course: ");
    console.println(nmea.getCourse() / 1000., 3);
    console.println("-----------------------");
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
      console.print(c);
      nmea.process(c);
    }
  }

}