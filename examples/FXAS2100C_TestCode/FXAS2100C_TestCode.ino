// ----------------------------------------------------------------------------
// 
// EXAMPLE SKETCH FOR THE GYROFXAS21002C SENSOR LIBRARY
// 
// Code By: Michael Wrona | GitHub: @michaelwro
// Created: 24 July 2020
// ----------------------------------------------------------------------------
/**
 * This is an example Arduino sketch that uses the GyroFXAS21002C gyroscope
 * sensor class. Gyro readings are output in [rad/s].
 * 
 * This code was tested with Adafruit's FXAS21002C/FXOS8700 I2C 9-DOF IMU.
 * 
 * I2C ARDUINO UNO WIRING
 * ----------------------
 * FXOS8700 VIN -> Arduino Uno 3.3V
 * FXOS8700 GND -> Arduino Uno Ground
 * FXOS8700 SDA -> Arduino Uno A4
 * FXOS8700 SCL -> Arduino Uno A5
 * 
 * References
 * ----------
 * ~ FXOX8700 + FXAS21002 9-DOF IMU (Adafruit):
 *     https://www.adafruit.com/product/3463
 */


// Include statements
#include <Arduino.h>
#include <Wire.h>
#include "GyroFXAS21002C.h"

// Sensor class
GyroSensorFXAS21002C Gyro = GyroSensorFXAS21002C(0x0021002C);

// Variables
unsigned long currMillis = 0;
unsigned long prevMillis = 0;
unsigned long samplePeriod = 20;  // Read every 20ms = 50Hz

float gx, gy, gz;

void setup() {
    Serial.begin(115200);  // Serial port
    while (Serial == false)
        delay(1);
    
    /**
     * Initialize sensor.
     * Measurement range options: GYRO_RNG_250DPS, GYRO_RNG_500DPS, GYRO_RNG_1000DPS,
     * and GYRO_RNG_2000DPS. GYRO_RNG_1000DPS default.
     */
    if (Gyro.InitializeSensor() == false) {
        Serial.println("Error initializing FXAS21002C. Check wiring.");
        while (1);
    }

    delay(500);
}


void loop() {

    currMillis = millis();  // Use millis() to time sensor readings
    if (currMillis - prevMillis >= samplePeriod) {
        prevMillis = currMillis;  // Replace time

        // Perform reading. Terminate if reading failed.
        if (Gyro.ReadSensor() == false) {
            return;
        }

        /**
         * Sensor data can be extracted two ways. The first assigns a value to a
         * variable via its pointer. The second is to use the public variables
         * stored in the class. You can use whatever method suits your project
         * better.
         * 
         * NOTE: The data type is float!
         */

        /* Method 1: Pointer method (preferred) */
        Gyro.GetGyroX(gx);
        Gyro.GetGyroY(gy);
        Gyro.GetGyroZ(gz);

        Serial.print("Gx: ");  // In [rad/s]
        Serial.print(gx);
        Serial.print("\tGy: ");
        Serial.print(gy);
        Serial.print("\tGz: ");
        Serial.println(gz);


        /* Method 2: Public class variables */
        // Serial.print("Gx: ");  // In [rad/s]
        // Serial.print(Gyro.gx);
        // Serial.print("\tGy: ");
        // Serial.print(Gyro.gy);
        // Serial.print("\tGz: ");
        // Serial.println(Gyro.gz);
    }
}
