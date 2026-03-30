#include <Arduino.h>
#include <SPI.h>

#include "SparkFun_BNO080_Arduino_Library.h"

namespace {

constexpr uint8_t kBnoChipSelectPin = 4;
constexpr uint8_t kBnoInterruptPin = 2;
constexpr uint8_t kBnoResetPin = 3;
BNO08x g_bno;

}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        // Wait for Serial on Teensy (optional)
    }

    Serial.println();
    Serial.println("BNO085 SPI Example (Teensy)");

    SPI.begin();
    if (!g_bno.beginSPI(kBnoChipSelectPin, kBnoInterruptPin, kBnoResetPin)) {
        Serial.println("BNO085 not detected. Check wiring/pins. Freezing...");
        while (true) {
        }
    }

    delay(10);
    g_bno.enableRotationVector();
}

void loop() {
    if (g_bno.wasReset()) {
        g_bno.enableRotationVector();
    }

    if (g_bno.getSensorEvent() && g_bno.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
        Serial.print(g_bno.getQuatReal(), 4);
        Serial.print(',');
        Serial.print(g_bno.getQuatI(), 4);
        Serial.print(',');
        Serial.print(g_bno.getQuatJ(), 4);
        Serial.print(',');
        Serial.println(g_bno.getQuatK(), 4);
    }

    delay(50);
}
