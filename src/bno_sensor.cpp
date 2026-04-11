#include "bno_sensor.h"

#include "bno055_sensor.h"
#include "bno085_sensor.h"
#include "settings.h"

namespace {

const char *ModelName(settings::sensors::bno::Model model) {
    switch (model) {
        case settings::sensors::bno::Model::Bno055:
            return "BNO055";
        case settings::sensors::bno::Model::Bno085:
            return "BNO085";
    }
    return "Unknown";
}

const char *TransportName(settings::sensors::bno::Transport transport) {
    switch (transport) {
        case settings::sensors::bno::Transport::I2c:
            return "I2C";
        case settings::sensors::bno::Transport::Spi:
            return "SPI";
    }
    return "Unknown";
}

}  // namespace

const char *BnoSensorModelName() {
    if (!settings::sensors::bno::kEnabled) {
        return "Disabled";
    }
    return ModelName(settings::sensors::bno::kModel);
}

const char *BnoSensorTransportName() {
    if (!settings::sensors::bno::kEnabled) {
        return "Disabled";
    }
    return TransportName(settings::sensors::bno::kTransport);
}

bool BnoSensorBegin() {
    if (!settings::sensors::bno::kEnabled) {
        return false;
    }
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorBegin();
        case settings::sensors::bno::Model::Bno085:
            return Bno085SensorBegin();
    }
    return false;
}

bool BnoSensorAcquire(SensorData &out) {
    if (!settings::sensors::bno::kEnabled) {
        return false;
    }
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorAcquire(out);
        case settings::sensors::bno::Model::Bno085:
            return Bno085SensorAcquire(out);
    }
    return false;
}

bool BnoSensorIsInitialized() {
    if (!settings::sensors::bno::kEnabled) {
        return false;
    }
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorIsInitialized();
        case settings::sensors::bno::Model::Bno085:
            return Bno085SensorIsInitialized();
    }
    return false;
}

BnoDiagnostics BnoSensorGetDiagnostics() {
    if (!settings::sensors::bno::kEnabled) {
        return BnoDiagnostics{};
    }
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorGetDiagnostics();
        case settings::sensors::bno::Model::Bno085: {
            const Bno085Diagnostics source = Bno085SensorGetDiagnostics();
            BnoDiagnostics diagnostics;
            diagnostics.transportReady = source.transportReady;
            diagnostics.hasAccel = source.hasAccel;
            diagnostics.hasGyro = source.hasGyro;
            diagnostics.hasMag = source.hasMag;
            diagnostics.hasQuaternion = source.hasQuaternion;
            diagnostics.hasBootstrapYpr = source.hasBootstrapYpr;
            diagnostics.lastAcquireFresh = source.lastAcquireFresh;
            for (int i = 0; i < 3; ++i) {
                diagnostics.yprDeg[i] = source.yprDeg[i];
                diagnostics.bootstrapYprDeg[i] = source.bootstrapYprDeg[i];
                diagnostics.accelBodyMps2[i] = source.accelBodyMps2[i];
                diagnostics.magBody[i] = source.magBody[i];
            }
            return diagnostics;
        }
    }
    return BnoDiagnostics{};
}

BnoSample BnoSensorGetSample() {
    if (!settings::sensors::bno::kEnabled) {
        return BnoSample{};
    }
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorGetSample();
        case settings::sensors::bno::Model::Bno085: {
            const Bno085Sample source = Bno085SensorGetSample();
            BnoSample sample;
            sample.hasAccel = source.hasAccel;
            sample.hasGyro = source.hasGyro;
            sample.hasQuaternion = source.hasQuaternion;
            sample.sampleMicros = source.sampleMicros;
            sample.accelMicros = source.accelMicros;
            sample.gyroMicros = source.gyroMicros;
            sample.quaternionMicros = source.quaternionMicros;
            for (int i = 0; i < 3; ++i) {
                sample.accel[i] = source.accel[i];
                sample.gyro[i] = source.gyro[i];
            }
            for (int i = 0; i < 4; ++i) {
                sample.quaternion[i] = source.quaternion[i];
            }
            return sample;
        }
    }
    return BnoSample{};
}
