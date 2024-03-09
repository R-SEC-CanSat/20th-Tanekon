#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Initialize the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055();

// Motor pins


void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Initialize BNO055 sensor
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055 sensor!");
        while (1);
    }
    
    // Set BNO055 sensor mode to IMU
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    
    // Set motor pins as output
    
}

void loop() {
    // Read the calibration status of the BNO055 sensor
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    
    // Check if the sensor is fully calibrated
    if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
        //ここにモーターの制御を書く
        
        // Print calibration complete message
        Serial.println("Calibration complete!");
        
        // Exit the loop
        while (1);
    }
    
    // Print calibration status
    Serial.print("Sys Cal: ");
    Serial.print(sys);
    Serial.print(" Gyro Cal: ");
    Serial.print(gyro);
    Serial.print(" Accel Cal: ");
    Serial.print(accel);
    Serial.print(" Mag Cal: ");
    Serial.println(mag);
    
    // Delay before checking calibration status again
    delay(1000);
}