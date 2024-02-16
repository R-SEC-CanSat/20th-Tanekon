
#include <Wire.h>
#include <SPI.h>

//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x42
#define I2C_DELAY 1

//I2C read data structures
char buff[80];
int idx = 0;
char  lat[9],lon[10];

//Read 80 bytes from I2C
void readI2C(char *inBuff)
{Wire.requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) 80);
  int i = 0;
  while (Wire.available()) {
    inBuff[i] = Wire.read();
    i++;
  }
}

void setup(void)
{
  Serial.begin(115200); // Serial
  Serial.println("Starting ...");
  Wire.begin(); // Wire


  delay(4000);

  //Reinitialize I2C after the reset
  Wire.begin();

  //clear i2c buffer
  char c;
  idx = 0;
  memset(buff, 0, 80);
  do {
    if (idx == 0) {
      readI2C(buff);
      delay(I2C_DELAY);
    }
    c = buff[idx];
    idx++;
    idx %= 80;
  }
  while ((uint8_t) c != 0xFF);

}
void GPS_data(){
  
  while(1){
    char c ;
    //改行文字で初期化したい
    if (idx == 0 ) {
      readI2C(buff);
      delay(I2C_DELAY);
      //Serial.print("readI2C");
      idx = 0;
    }
    //Fetch the character one by one
    c = buff[idx];
    idx++;
    idx %= 80;
    //If we have a valid character pass it to the library
    if ((uint8_t) c != 0xFF) {
      Serial.print(c);
      //GGAならば緯度経度を取得する
      if (c == '$' && idx < 40) {
        if(buff[idx+2] == 'G'){
          if(buff[idx+3] == 'G'){
            if(buff[idx+4] == 'A'){
              for(int i = 0; i < 4; i++){
                lat[i] = buff[idx+16+i];
              }
              for(int i = 0; i < 5; i++){
                //小数点は除外する
                lat[i + 4] = buff[idx+21+i];
              }
              for(int i = 0; i < 5; i++){
                //小数点は除外する
                lon[i] = buff[idx+29+i];
              }
              for(int i = 0; i < 5; i++){
                //小数点は除外する
                lon[i + 5] = buff[idx+35+i];
              }
              
              break;
              
            }
          }
        }
      }
    
    }
  }
}
void loop(void)
{
  GPS_data();
  
  long mlat = atol(lat);
  double latitude = (double)mlat * 1.0E-7;
  long mlon = atol(lon);
  double longitude = mlon * 1.0E-7;
  Serial.print("latitude_deg: ");
  Serial.println(latitude,7);
  Serial.println();
  Serial.print("longitude_deg: ");
  Serial.println(longitude,7);  
  Serial.println();
}