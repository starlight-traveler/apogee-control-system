#pragma once

#include "flight_computer.h"

/**
 * @brief diagnostics for the Ellipse 2.0 comparison INS path.
 *
 * The Ellipse path is a high-quality external reference, but it still enters
 * the same selector rules as other rails: fresh data, known body-frame mapping,
 * and explicit availability flags. These diagnostics make it clear whether the
 * INS is contributing real-time measurements or only cached comparison data.
 */
struct Ellipse20Diagnostics {
    bool initialized = false;
    // Report availability from the external INS.
    bool hasImu = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool alignmentReady = false;
    bool hasYpr = false;
    // Fresh/cache flags are important because serial INS frames can arrive slower than the loop.
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    uint32_t lastSampleMicros = 0;
    // Body-frame vectors let logs compare the INS rail directly against onboard IMUs.
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float gyroBodyRadPerSec[3] = {0.0f, 0.0f, 0.0f};
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
};

/// @brief initializes the Ellipse serial/INS path.
bool Ellipse20SensorBegin();
/// @brief gives the INS adapter current flight phase for phase-dependent trust rules.
void Ellipse20SensorSetFlightStatus(FlightStatus status);
/// @brief applies external trust from cross-checking against onboard IMU rails.
void Ellipse20SensorSetCrossCheckTrust(float trust);
/// @brief services incoming serial frames outside the main acquire call.
void Ellipse20SensorService();
/// @brief publishes the latest fresh Ellipse sample into `SensorData`.
bool Ellipse20SensorAcquire(SensorData &out);
/// @brief returns true once the Ellipse adapter is initialized.
bool Ellipse20SensorIsInitialized();
/// @brief returns current Ellipse health and last-sample diagnostics.
Ellipse20Diagnostics Ellipse20SensorGetDiagnostics();
