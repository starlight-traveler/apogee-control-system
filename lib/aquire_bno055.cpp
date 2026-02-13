#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

namespace {

constexpr uint8_t kBnoI2cAddress = 0x28;
Adafruit_BNO055 g_bno(55, kBnoI2cAddress);

}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        // Wait for Serial on Teensy (optional)
    }

    Serial.println();
    Serial.println("BNO055 I2C Example (Teensy)");

    Wire.begin();
    if (!g_bno.begin()) {
        Serial.println("BNO055 not detected. Check wiring/address. Freezing...");
        while (true) {
        }
    }

    delay(10);
    g_bno.setExtCrystalUse(true);
}

void loop() {
    const imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print(euler.x(), 1);
    Serial.print(',');
    Serial.print(euler.y(), 1);
    Serial.print(',');
    Serial.println(euler.z(), 1);

    delay(50);
}
