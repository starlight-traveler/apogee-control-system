#include <Arduino.h>
#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Install via Library Manager

BNO080 myIMU;

// Adjust these to match your wiring
const uint8_t imuCSPin = 36;
const uint8_t imuWAKPin = 32;
const uint8_t imuINTPin = 33;
const uint8_t imuRSTPin = 35;

// Flag set from ISR when INT goes low (data ready)
volatile bool imuIntFlag = false;

// Simple, fast ISR: just set a flag
void imuIntISR()
{
    imuIntFlag = true;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000)
    {
        // Wait for Serial on Teensy (optional)
    }

    Serial.println();
    Serial.println("BNO080 SPI + Interrupt Example (Teensy)");

    // Make sure INT is configured as input before attachInterrupt
    pinMode(imuINTPin, INPUT_PULLUP); // BNO080 INT is usually active-low; pullup is typical

    // Initialize BNO080 over SPI1
    // last arg is SPI port: SPI, SPI1, SPI2 depending on your Teensy model
    if (!myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 30000000UL, SPI1))
    {
        Serial.println("BNO080 over SPI not detected. Check wiring. Freezing...");
        while (1)
        {
            // Do nothing
        }
    }

    // Optional: debug from the SparkFun driver
    // myIMU.enableDebugging(Serial);

    // Enable rotation vector at 50ms = 20 Hz
    myIMU.enableRotationVector(50);

    Serial.println("Rotation vector enabled");
    Serial.println("Output: roll, pitch, yaw (deg)");

    // Attach hardware interrupt on INT pin.
    // BNO080 INT is active LOW when data is ready, so trigger on FALLING.
    attachInterrupt(digitalPinToInterrupt(imuINTPin), imuIntISR, FALLING);
}

void loop()
{
    // Copy and clear the flag atomically
    bool doRead = false;
    noInterrupts();
    if (imuIntFlag)
    {
        imuIntFlag = false;
        doRead = true;
    }
    interrupts();

    if (doRead)
    {
        // There may be more than one report queued; drain them all
        while (myIMU.dataAvailable())
        {
            float roll = myIMU.getRoll() * 180.0f / PI;
            float pitch = myIMU.getPitch() * 180.0f / PI;
            float yaw = myIMU.getYaw() * 180.0f / PI;

            Serial.print(roll, 1);
            Serial.print(',');
            Serial.print(pitch, 1);
            Serial.print(',');
            Serial.print(yaw, 1);
            Serial.println();
        }
    }

    // Your other application code can run here without constantly polling the IMU
    // ...
}
