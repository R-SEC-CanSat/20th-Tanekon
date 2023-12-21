/*
Hardware connections:
  BME280 -> Arduino
  GND -> GND
  3.3 -> 3.3
  SDA -> A4
  SCL -> A5
*/

#include <Wire.h>
#include "SparkFunBME280.h"
#include <M5Core2.h>

BME280 sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.beginI2C();  
}

void loop() {
  Serial.print("Temp: ");
  Serial.print(sensor.readTempC(), 2);

  Serial.print(" °C, Humidity: ");
  Serial.print(sensor.readFloatHumidity(), 2);

  Serial.print(" %, Pressure: ");
  Serial.print(sensor.readFloatPressure() / 100.0, 1);
  Serial.println(" hPa");

  delay(5000);
}
