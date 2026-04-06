// Reefwing-AHRS test with Reefwing-LSM9DS1 (I2C)
// Uses Reefwing libraries for both sensor reading and AHRS fusion.
// Compare the Reefwing AHRS output against the BNO085 reference.
//
// NOTE: This uses I2C, not SPI. Wire the LSM9DS1 to I2C pins.
// BOARD_NAME is defined via build flags in platformio.ini.

#include <Arduino.h>
#include <Wire.h>
#include <ReefwingLSM9DS1.h>
#include <ReefwingAHRS.h>

#ifndef ACS_BUILD_REEFWING_LSM9DS1_TEST
#error "Use the reefwing_lsm9ds1_test PlatformIO environment to build this target."
#endif

namespace {

// Timing
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kUpdateIntervalMs = 10;   // 100 Hz update rate
constexpr uint32_t kPrintIntervalMs = 100;   // 10 Hz print rate

// Magnetic declination (same as main firmware)
constexpr float kMagDeclinationDeg = -14.84f;

// LSM9DS1 mount rotation: X inverted relative to BNO frame
// Transform: BNO_X = -LSM_X, BNO_Y = LSM_Y, BNO_Z = LSM_Z
constexpr float kMountRotation[3][3] = {
    {-1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

ReefwingLSM9DS1 g_imu;
ReefwingAHRS g_ahrs;
SensorData g_sensorData;

uint32_t g_lastUpdateMs = 0;
uint32_t g_lastPrintMs = 0;
bool g_initialized = false;

// Track current beta value since there's no getter
float g_currentBeta = 0.1f;

// Debug: track raw mag values to diagnose yaw drift
float g_rawMag[3] = {0.0f, 0.0f, 0.0f};
bool g_magValid = false;

void ApplyMountRotation(float &x, float &y, float &z) {
    float in[3] = {x, y, z};
    float out[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        out[i] = kMountRotation[i][0] * in[0] +
                 kMountRotation[i][1] * in[1] +
                 kMountRotation[i][2] * in[2];
    }
    x = out[0];
    y = out[1];
    z = out[2];
}

bool InitializeSensor() {
    Serial.println("Initializing LSM9DS1 via I2C (Reefwing library)...");

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast mode

    // Initialize the IMU
    g_imu.begin();

    // Check if connected
    if (!g_imu.connected()) {
        Serial.println("ERROR: LSM9DS1 not detected on I2C!");
        Serial.println("  Check wiring: SDA, SCL, VCC, GND");
        Serial.println("  Default I2C addresses: AG=0x6B, Mag=0x1E");
        return false;
    }

    Serial.print("  Gyro WHO_AM_I: 0x");
    Serial.println(g_imu.whoAmIGyro(), HEX);
    Serial.print("  Mag WHO_AM_I: 0x");
    Serial.println(g_imu.whoAmIMag(), HEX);

    // Configure for MAX ranges as requested
    g_imu.setGyroScale(GyroScale::FS_2000DPS);   // Max: 2000 dps
    g_imu.setAccelScale(AccelScale::FS_XL_16G);  // Max: 16g
    g_imu.setMagScale(MagScale::FS_16G);         // Max: 16 Gauss

    // Set high sample rates
    g_imu.setGyroODR(GyroODR::GODR_952Hz);
    g_imu.setAccelODR(AccelODR::AODR_952Hz);
    g_imu.setMagODR(MagODR::MODR_80Hz);

    // Enable temperature compensation for magnetometer
    g_imu.enableMagTempComp(true);

    // Start continuous sampling
    g_imu.start();

    Serial.println("LSM9DS1 initialized with MAX ranges:");
    Serial.println("  Accel: +/- 16g");
    Serial.println("  Gyro:  +/- 2000 dps");
    Serial.println("  Mag:   +/- 16 Gauss");
    Serial.println("  ODR:   952 Hz (accel/gyro), 80 Hz (mag)");

    return true;
}

void InitializeAHRS() {
    Serial.println("Initializing Reefwing AHRS...");

    g_ahrs.begin();

    // Configure for 9-DOF operation with magnetometer
    g_ahrs.setDOF(DOF::DOF_9);
    g_ahrs.setImuType(ImuType::LSM9DS1);

    // Start with Madgwick filter
    g_ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);

    // Set magnetic declination
    g_ahrs.setDeclination(kMagDeclinationDeg);

    // Madgwick tuning - higher beta = faster convergence but more noise
    g_currentBeta = 0.1f;
    g_ahrs.setBeta(g_currentBeta);

    Serial.println("Reefwing AHRS initialized:");
    Serial.print("  Fusion: MADGWICK, DOF: 9, Declination: ");
    Serial.print(kMagDeclinationDeg);
    Serial.println(" deg");
}

void ReadSensors() {
    // Read scaled sensor data from Reefwing library
    ScaledData accel = g_imu.readAccel();
    ScaledData gyro = g_imu.readGyro();
    ScaledData mag = g_imu.readMag();

    // Store raw mag for debugging
    g_rawMag[0] = mag.sx;
    g_rawMag[1] = mag.sy;
    g_rawMag[2] = mag.sz;

    // Check if mag data is valid (not all zeros)
    float magMagnitude = sqrtf(mag.sx * mag.sx + mag.sy * mag.sy + mag.sz * mag.sz);
    g_magValid = (magMagnitude > 0.01f);

    // Apply mount rotation to align with BNO reference frame
    float ax = accel.sx, ay = accel.sy, az = accel.sz;
    float gx = gyro.sx, gy = gyro.sy, gz = gyro.sz;
    float mx = mag.sx, my = mag.sy, mz = mag.sz;

    ApplyMountRotation(ax, ay, az);
    ApplyMountRotation(gx, gy, gz);
    ApplyMountRotation(mx, my, mz);

    // Set sensor data for AHRS
    // Reefwing expects: accel in g, gyro in dps, mag in gauss
    g_sensorData.ax = ax;
    g_sensorData.ay = ay;
    g_sensorData.az = az;
    g_sensorData.gx = gx;
    g_sensorData.gy = gy;
    g_sensorData.gz = gz;
    g_sensorData.mx = mx;
    g_sensorData.my = my;
    g_sensorData.mz = mz;
    g_sensorData.gTimeStamp = micros();
}

void UpdateAHRS() {
    g_ahrs.setData(g_sensorData);
    g_ahrs.update();
}

void PrintResults() {
    // Access the public angles member directly
    float yaw = g_ahrs.angles.yaw;
    float pitch = g_ahrs.angles.pitch;
    float roll = g_ahrs.angles.roll;

    // Normalize yaw to 0-360
    while (yaw < 0.0f) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;

    Serial.print("YPR: ");
    Serial.print(yaw, 1);
    Serial.print(", ");
    Serial.print(pitch, 1);
    Serial.print(", ");
    Serial.print(roll, 1);

    // Print mag data to debug yaw drift issue
    Serial.print("  | Mag(raw): ");
    Serial.print(g_rawMag[0], 3);
    Serial.print(", ");
    Serial.print(g_rawMag[1], 3);
    Serial.print(", ");
    Serial.print(g_rawMag[2], 3);

    float magMag = sqrtf(g_rawMag[0]*g_rawMag[0] + g_rawMag[1]*g_rawMag[1] + g_rawMag[2]*g_rawMag[2]);
    Serial.print(" |");
    Serial.print(magMag, 3);
    Serial.print("| ");
    Serial.print(g_magValid ? "OK" : "BAD");

    Serial.println();
}

void PrintDiagnostics() {
    Serial.println();
    Serial.println("=== Magnetometer Diagnostics ===");
    Serial.print("Raw Mag X: "); Serial.println(g_rawMag[0], 4);
    Serial.print("Raw Mag Y: "); Serial.println(g_rawMag[1], 4);
    Serial.print("Raw Mag Z: "); Serial.println(g_rawMag[2], 4);
    float magnitude = sqrtf(g_rawMag[0]*g_rawMag[0] + g_rawMag[1]*g_rawMag[1] + g_rawMag[2]*g_rawMag[2]);
    Serial.print("Magnitude: "); Serial.println(magnitude, 4);
    Serial.print("Valid: "); Serial.println(g_magValid ? "YES" : "NO");

    // Calculate heading from mag only (for debugging)
    float heading = atan2f(g_rawMag[1], g_rawMag[0]) * 57.2957795f;
    if (heading < 0) heading += 360.0f;
    Serial.print("Mag-only heading: "); Serial.println(heading, 1);

    Serial.println();
    Serial.println("If mag magnitude is 0 or very small, the magnetometer");
    Serial.println("is not being read properly. Check I2C wiring.");
    Serial.println();
    Serial.println("If yaw drifts to a fixed value (like 287 deg), the");
    Serial.println("magnetometer needs calibration (hard/soft iron).");
    Serial.println();
}

void PrintMenu() {
    Serial.println();
    Serial.println("=== Reefwing AHRS Test - LSM9DS1 (I2C) ===");
    Serial.println("Commands:");
    Serial.println("  m - Switch to MADGWICK filter");
    Serial.println("  h - Switch to MAHONY filter");
    Serial.println("  c - Switch to COMPLEMENTARY filter");
    Serial.println("  k - Switch to EXTENDED_KALMAN filter");
    Serial.println("  + - Increase filter gain (beta/Kp)");
    Serial.println("  - - Decrease filter gain (beta/Kp)");
    Serial.println("  d - Print magnetometer diagnostics");
    Serial.println("  r - Reset AHRS");
    Serial.println("  ? - Print this menu");
    Serial.println();
}

void HandleSerialInput() {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
        case 'm':
        case 'M':
            g_ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
            Serial.println("Switched to MADGWICK filter");
            break;

        case 'h':
        case 'H':
            g_ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
            Serial.println("Switched to MAHONY filter");
            break;

        case 'c':
        case 'C':
            g_ahrs.setFusionAlgorithm(SensorFusion::COMPLEMENTARY);
            Serial.println("Switched to COMPLEMENTARY filter");
            break;

        case 'k':
        case 'K':
            g_ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);
            Serial.println("Switched to EXTENDED_KALMAN filter");
            break;

        case '+':
        case '=':
            g_currentBeta *= 1.2f;
            g_ahrs.setBeta(g_currentBeta);
            Serial.print("Beta increased to: ");
            Serial.println(g_currentBeta, 4);
            break;

        case '-':
        case '_':
            g_currentBeta *= 0.8f;
            g_ahrs.setBeta(g_currentBeta);
            Serial.print("Beta decreased to: ");
            Serial.println(g_currentBeta, 4);
            break;

        case 'd':
        case 'D':
            PrintDiagnostics();
            break;

        case 'r':
        case 'R':
            g_ahrs.reset();
            Serial.println("AHRS reset");
            break;

        case '?':
            PrintMenu();
            break;
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection
    }

    Serial.println();
    Serial.println("==========================================");
    Serial.println("  Reefwing AHRS Test - LSM9DS1 (I2C)");
    Serial.println("  MAX RANGES: 16g / 2000dps / 16Gauss");
    Serial.println("==========================================");
    Serial.println();

    if (!InitializeSensor()) {
        Serial.println("FATAL: Sensor initialization failed!");
        Serial.println("This test requires I2C connection to LSM9DS1.");
        while (true) {
            delay(1000);
        }
    }

    InitializeAHRS();
    PrintMenu();

    g_initialized = true;
    g_lastUpdateMs = millis();
    g_lastPrintMs = millis();
}

void loop() {
    if (!g_initialized) return;

    uint32_t now = millis();

    // Update AHRS at 100 Hz
    if (now - g_lastUpdateMs >= kUpdateIntervalMs) {
        g_lastUpdateMs = now;
        ReadSensors();
        UpdateAHRS();
    }

    // Print results at 10 Hz
    if (now - g_lastPrintMs >= kPrintIntervalMs) {
        g_lastPrintMs = now;
        PrintResults();
    }

    // Handle serial commands
    HandleSerialInput();
}
