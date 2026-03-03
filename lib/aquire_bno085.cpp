#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO08x.h>
#include <sh2.h>

namespace {

constexpr uint8_t kBnoI2cAddress = 0x28;
Adafruit_BNO08x g_bno(-1);

}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        // Wait for Serial on Teensy (optional)
    }

    Serial.println();
    Serial.println("BNO085 I2C Example (Teensy)");

    Wire.begin();
    if (!g_bno.begin_I2C(kBnoI2cAddress, &Wire)) {
        Serial.println("BNO085 not detected. Check wiring/address. Freezing...");
        while (true) {
        }
    }

    delay(10);
    g_bno.enableReport(SH2_ROTATION_VECTOR, 20000);
}

void loop() {
    sh2_SensorValue_t sensorValue;
    if (g_bno.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        Serial.print(sensorValue.un.rotationVector.real, 4);
        Serial.print(',');
        Serial.print(sensorValue.un.rotationVector.i, 4);
        Serial.print(',');
        Serial.print(sensorValue.un.rotationVector.j, 4);
        Serial.print(',');
        Serial.println(sensorValue.un.rotationVector.k, 4);
    }

    delay(50);
}
