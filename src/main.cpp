#include <Arduino.h>

#include "bno085_sensor.h"
#include "bmp581_sensor.h"
#include "data_logger.h"
#include "flight_computer.h"

static bool AcquireSensorData(SensorData &data) {
    bool hasImu = Bno085SensorAcquire(data);
    bool hasAltimeter = Bmp581SensorAcquire(data);
    return hasImu || hasAltimeter;
}

static FlightComputer flightComputer;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
    }

    if (!Bno085SensorBegin()) {
        Serial.println("Failed to initialize BNO085 sensor.");
        while (true) {
            delay(100);
        }
    }

    if (!Bmp581SensorBegin()) {
        Serial.println("Failed to initialize BMP581 sensor.");
        while (true) {
            delay(100);
        }
    }

    if (!DataLoggerBegin()) {
        Serial.println("Sensor logging is disabled.");
        while (true)
        {
            delay(100);
        }
    }

    const EnvironmentModel::Config environmentConfig;
    ApogeeVehicleParameters vehicleParameters;

    const float sigmaAccelXY = 0.5f;
    const float sigmaAccelZ = 0.5f;
    const float sigmaAltimeter = 0.5f;
    const float processXY = 0.5f;
    const float processZ = 1.0f;
    const float apogeeTargetMeters = 1550.0f;

    flightComputer.Begin(sigmaAccelXY,
                         sigmaAccelZ,
                         sigmaAltimeter,
                         processXY,
                         processZ,
                         apogeeTargetMeters,
                         environmentConfig,
                         vehicleParameters);

    Serial.println("Flight computer initialized.");
}

void loop() {
    DataLoggerService();

    SensorData data;
    if (!AcquireSensorData(data)) {
        delay(1);
        return;
    }

    DataLoggerLog(data);

    FilteredState state;
    if (flightComputer.Update(data, state) && Serial) {
        Serial.print("t=");
        Serial.print(state.time, 3);
        Serial.print(" z=");
        Serial.print(state.position[2], 2);
        Serial.print(" vz=");
        Serial.print(state.velocity[2], 2);
        Serial.print(" az=");
        Serial.print(state.acceleration[2], 2);
        Serial.print(" apg=");
        Serial.print(state.apogeeEstimate, 2);
        Serial.print(" status=");
        Serial.println(FlightStatusToString(flightComputer.Status()));
    }

    DataLoggerService();
}
