// ----------------------------------------------------------------------------
// FXAS21002C GYROSCOPE SENSOR LIBRARY
// A custom sensor library for the FXAS21002C I2C gyroscope sensor. Inspired by
// Adafruit's FXAS21002C Library (see Resources).
// 
// Code By: Michael Wrona | B.S. Aerospace Engineering
// Created: 25 July 2020
// ----------------------------------------------------------------------------
/**
 * This is the sensor library for the FXAS21002C 3-axis gyroscope sensor. This
 * library was inspired by Adafruit's FXAS21002C Library (see Resources). Tested
 * and verified with Adafruit's FXAS21002C/FXOS8700 9-DOF IMU and an Arduino Uno.
 * 
 * Resources
 * ---------
 * ~ Adafruit FXAS21002C Sensor Library (GitHub):
 *     https://github.com/adafruit/Adafruit_FXAS21002C
 * ~ FXOX8700 + FXAS21002 9-DOF IMU (Adafruit):
 *     https://www.adafruit.com/product/3463
 */


#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>
#include "GyroFXAS21002C.h"



// ------------------------------------
// Public methods
// ------------------------------------

// ----------------------------------------------------------------------------
// 
// ----------------------------------------------------------------------------
/**
 * Constructor for FXAS21002C gyro sensor.
 * 
 * @param gyroID  ID for sensor, 0x0021002C
 */
GyroSensorFXAS21002C::GyroSensorFXAS21002C(int32_t inputGyroID)
{
    this->gyroID = inputGyroID;
}

// ----------------------------------------------------------------------------
// InitializeSensor(GyroRanges_t range)
// ----------------------------------------------------------------------------
/**
 * Initialize and configure gyroscope sensor. If unspecified, the default gyro
 * measurement rage is GYRO_RNG_1000DPS. Options include: GYRO_RNG_250DPS,
 * GYRO_RNG_500DPS, GYRO_RNG_1000DPS, and GYRO_RNG_2000DPS
 * 
 * @return  True if successful, false if failed.
 */
bool GyroSensorFXAS21002C::InitializeSensor(GyroRanges_t rng)
{
    uint8_t ctrlReg0;
    uint8_t connectedSensorID;

    Wire.begin();  // Init. I2C
    this->gyroRange = rng;  // Set range
    
    // Clear raw data
    this->gx = 0.0F;
    this->gy = 0.0F;
    this->gz = 0.0F;

    /* Check to make sure the ID register on the sensor matches the
    expected FXAS21002C ID. */
    connectedSensorID = this->I2Cread8(GYRO_REG_ID);
    if (connectedSensorID != FXAS21002C_ID)
        return false;  // ID's dont match!
    
    /* Set sensor configuration parameters */
    ctrlReg0 = 0x00;
    switch (this->gyroRange)
    {
        case GYRO_RNG_250DPS:
            ctrlReg0 = 0x03;
            break;
        case GYRO_RNG_500DPS:
            ctrlReg0 = 0x02;
            break;
        case GYRO_RNG_1000DPS:
            ctrlReg0 = 0x01;
            break;
        case GYRO_RNG_2000DPS:
            ctrlReg0 = 0x00;
            break;
    }

    /* Reset sensor, then switch to active mode to configure */
    this->I2Cwrite8(GYRO_REG_CTRL1, 0x00);  // Stby
    this->I2Cwrite8(GYRO_REG_CTRL1, (1 << 6));  // Reset
    this->I2Cwrite8(GYRO_REG_CTRL0, ctrlReg0);  // Set sensitivity
    this->I2Cwrite8(GYRO_REG_CTRL1, 0x0E);  // Active
    delay(100);  // Short delay

    return true;
}


// ----------------------------------------------------------------------------
// ReadSensor()
// ----------------------------------------------------------------------------
/**
 * Read gyroscope data from device registers. Computes gyro readings in [rad/s]
 * 
 * @return  True if successful, false if failed.
 */
bool GyroSensorFXAS21002C::ReadSensor()
{
    this->gx = 0.0f;
    this->gy = 0.0f;
    this->gz = 0.0f;
    
    // Read 7 bytes from sensor
    Wire.beginTransmission((byte)FXAS21002C_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(GYRO_REG_STATUS | 0x80);
    #else
        Wire.send(GYRO_REG_STATUS | 0x80);
    #endif
    Wire.endTransmission();
    Wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)7);

    #if ARDUINO >= 100
        uint8_t status = Wire.read();
        uint8_t xhi = Wire.read();
        uint8_t xlo = Wire.read();
        uint8_t yhi = Wire.read();
        uint8_t ylo = Wire.read();
        uint8_t zhi = Wire.read();
        uint8_t zlo = Wire.read();
    #else
        uint8_t status = Wire.receive();
        uint8_t xhi = Wire.receive();
        uint8_t xlo = Wire.receive();
        uint8_t yhi = Wire.receive();
        uint8_t ylo = Wire.receive();
        uint8_t zhi = Wire.receive();
        uint8_t zlo = Wire.receive();
    #endif

    // Shift values to make proper integer
    this->gx = (int16_t)((xhi << 8) | xlo);
    this->gy = (int16_t)((yhi << 8) | ylo);
    this->gz = (int16_t)((zhi << 8) | zlo);

    // Convert int readings to floats [dps] depending on sensitivity
    switch (this->gyroRange)
    {
        case GYRO_RNG_250DPS:
            this->gx *= GYRO_SENS_250;
            this->gy *= GYRO_SENS_250;
            this->gz *= GYRO_SENS_250;
            break;
        case GYRO_RNG_500DPS:
            this->gx *= GYRO_SENS_500;
            this->gy *= GYRO_SENS_500;
            this->gz *= GYRO_SENS_500;
            break;
        case GYRO_RNG_1000DPS:
            this->gx *= GYRO_SENS_1000;
            this->gy *= GYRO_SENS_1000;
            this->gz *= GYRO_SENS_1000;
            break;
        case GYRO_RNG_2000DPS:
            this->gx *= GYRO_SENS_2000;
            this->gy *= GYRO_SENS_2000;
            this->gz *= GYRO_SENS_2000;
            break;
    }

   // Convert [dps] to [rad/s]
   this->gx *= (float)DEG_TO_RAD;  // DEG_T0_RAD defined in Arduino.h
   this->gy *= (float)DEG_TO_RAD;
   this->gz *= (float)DEG_TO_RAD;

    return true;
}


// ----------------------------------------------------------------------------
// GetGyroX(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Assign gyro x-measurement [rad/s] to a float pointer.
 */
void GyroSensorFXAS21002C::GetGyroX(float &ptrOut)
{
    ptrOut = this->gx;
}


// ----------------------------------------------------------------------------
// GetGyroY(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Assign gyro y-measurement [rad/s] to a float pointer.
 */
void GyroSensorFXAS21002C::GetGyroY(float &ptrOut)
{
    ptrOut = this->gy;
}


// ----------------------------------------------------------------------------
// GetGyroZ(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Assign gyro z-measurement [rad/s] to a float pointer.
 */
void GyroSensorFXAS21002C::GetGyroZ(float &ptrOut)
{
    ptrOut = this->gz;
}



// ------------------------------------
// Private methods
// ------------------------------------


// ----------------------------------------------------------------------------
// I2Cwrite8(byte regOfInterest, byte valToWrite)
// ----------------------------------------------------------------------------
/**
 * Write to device register over I2C.
 * 
 * @param regOfInterest  Register address on device.
 * @param valToWrite     Value to write to register.
 */
void GyroSensorFXAS21002C::I2Cwrite8(byte regOfInterest, byte valToWrite)
{
    // Init. communication
    Wire.beginTransmission(FXAS21002C_ADDRESS);
    #if ARDUINO >= 100
        Wire.write((uint8_t)regOfInterest);
        Wire.write((uint8_t)valToWrite);
    #else
        Wire.send(regOfInterest);
        Wire.send(valToWrite);
    #endif
    Wire.endTransmission();
}


// ----------------------------------------------------------------------------
// I2Cread8(byte regOfInterest)
// ----------------------------------------------------------------------------
/**
 * Read register value from I2C device.
 * 
 * @param regOfInterest  Register address on device.
 * @return               Value/data in register.
 */
byte GyroSensorFXAS21002C::I2Cread8(byte regOfInterest)
{
    byte val;

    // Init. communication
    Wire.beginTransmission((byte)FXAS21002C_ADDRESS);
    #if ARDUINO >= 100
        Wire.write((uint8_t)regOfInterest);
    #else
        Wire.send(regOfInterest);
    #endif

    // Check for failure
    if (Wire.endTransmission(false) != 0)
        return 0;
    
    // Read register
    Wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)1);
    #if ARDUINO >= 100
        val = Wire.read();
    #else
        val = Wire.receive();
    #endif

    return val;
}
