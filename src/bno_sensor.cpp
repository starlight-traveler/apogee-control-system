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
    return ModelName(settings::sensors::bno::kModel);
}

const char *BnoSensorTransportName() {
    return TransportName(settings::sensors::bno::kTransport);
}

bool BnoSensorBegin() {
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorBegin();
        case settings::sensors::bno::Model::Bno085:
            return Bno085SensorBegin();
    }
    return false;
}

bool BnoSensorAcquire(SensorData &out) {
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorAcquire(out);
        case settings::sensors::bno::Model::Bno085:
            return Bno085SensorAcquire(out);
    }
    return false;
}

bool BnoSensorIsInitialized() {
    switch (settings::sensors::bno::kModel) {
        case settings::sensors::bno::Model::Bno055:
            return Bno055SensorIsInitialized();
        case settings::sensors::bno::Model::Bno085:
            return Bno085SensorIsInitialized();
    }
    return false;
}
