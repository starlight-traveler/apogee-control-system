#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "constants.h"
#include "environment_model.h"
#include "math_utils.h"

/// \file apogee_model.h
/// \brief Ballistic apogee prediction and CFD lookup tables.

/// \brief C++ equivalent to apogee.py and apogee_lib.py focused on ballistic prediction.

/// \brief Vehicle parameters used by the apogee predictor.
struct ApogeeVehicleParameters {
    /// \brief Center of pressure offset from CG in meters.
    float centerOfPressureOffsetMeters = 0.0f;  // cp_cg in Python (m)
    /// \brief Moment of inertia about the pitch axis in kg*m^2.
    float momentOfInertia = 1.0f;               // kg·m^2
    /// \brief Dry mass in kilograms.
    float dryMass = 1.0f;                       // kg
    /// \brief ACS angle in degrees for CFD table lookup.
    float acsAngleDeg = 0.0f;                   // ACS angle input for CFD table (deg)
};

/// \brief State used by the apogee predictor integrator.
struct ApogeeState {
    /// \brief Altitude in meters.
    float altitudeMeters = 0.0f;
    /// \brief Horizontal distance in meters.
    float horizontalDistanceMeters = 0.0f;
    /// \brief Vertical velocity in meters per second.
    float verticalVelocity = 0.0f;
    /// \brief Horizontal velocity in meters per second.
    float horizontalVelocity = 0.0f;
    /// \brief Zenith angle in radians.
    float zenith = 0.0f;
    /// \brief Angular velocity in radians per second.
    float angularVelocity = 0.0f;
};

/// \brief Predicts apogee using environment and CFD-based aero forces.
class ApogeePredictor {
  public:
    /// \brief Construct with environment and vehicle parameters.
    /// \param environment Environment model reference.
    /// \param vehicle Vehicle parameters.
    ApogeePredictor(const EnvironmentModel &environment, const ApogeeVehicleParameters &vehicle)
        : environment_(&environment), vehicle_(vehicle) {}

    /// \brief Construct with default parameters.
    ApogeePredictor() = default;

    /// \brief Update the environment pointer.
    /// \param environment Environment model reference.
    void SetEnvironment(const EnvironmentModel &environment) { environment_ = &environment; }
    /// \brief Update vehicle parameters.
    /// \param vehicle Vehicle parameters to copy.
    void SetVehicleParameters(const ApogeeVehicleParameters &vehicle) { vehicle_ = vehicle; }

    /// \brief Set integration time step in seconds.
    /// \param dt Time step in seconds.
    void SetTimeStep(float dt) { timeStep_ = dt; }
    /// \brief Load CFD lookup table from CSV.
    /// \param path Path to CSV file.
    /// \return True if loaded successfully.
    bool LoadCfdTable(const char *path) { return cfd_.LoadFromCsv(path); }

    /// \brief Predict apogee altitude from an initial state.
    /// \param initialState Initial kinematic state.
    /// \return Predicted apogee altitude in meters.
    float PredictApogee(const ApogeeState &initialState) {
        if (!std::isfinite(initialState.verticalVelocity) || !std::isfinite(initialState.altitudeMeters)) {
            return initialState.altitudeMeters;
        }
        if (initialState.verticalVelocity <= minVerticalVelocityForPrediction_) {
            return initialState.altitudeMeters;
        }
        ApogeeState state = initialState;
        float apogee = state.altitudeMeters;
        const int maxIterations = 2000;
        for (int i = 0; i < maxIterations; ++i) {
            if (state.verticalVelocity <= 0.0f) {
                break;
            }
            state = IntegrateStep(state, timeStep_);
            if (state.altitudeMeters > apogee) {
                apogee = state.altitudeMeters;
            }
        }
        return apogee;
    }

  private:
    /// \brief Derivative terms used in RK4 integration.
    struct Derivative {
        /// \brief Altitude rate in meters per second.
        float altitudeRate;
        /// \brief Horizontal rate in meters per second.
        float horizontalRate;
        /// \brief Vertical acceleration in meters per second squared.
        float verticalAcceleration;
        /// \brief Horizontal acceleration in meters per second squared.
        float horizontalAcceleration;
        /// \brief Zenith rate in radians per second.
        float zenithRate;
        /// \brief Angular acceleration in radians per second squared.
        float angularAcceleration;
    };

    /// \brief Acceleration results from aero and gravity.
    struct AccelResult {
        /// \brief Linear acceleration vector.
        math_utils::Vec3 linear;  // x: vertical, y: horizontal, z: unused
        /// \brief Angular acceleration.
        float angular;
    };

    /// \brief CFD lookup table with trilinear interpolation.
    struct CfdLookup {
        /// \brief Single CFD record from the CSV file.
        struct Record {
            /// \brief ACS angle in degrees.
            float acs = 0.0f;
            /// \brief Angle of attack in degrees.
            float atk = 0.0f;
            /// \brief Mach number.
            float mach = 0.0f;
            /// \brief Axial force coefficient.
            float axial = 0.0f;
            /// \brief Normal force coefficient.
            float normal = 0.0f;
        };

        /// \brief Load CFD records from a CSV file.
        /// \param path Path to CSV file.
        /// \return True if records were loaded successfully.
        bool LoadFromCsv(const char *path) {
            if (path == nullptr || path[0] == '\0') {
                return false;
            }

            std::ifstream file(path);
            if (!file.is_open()) {
                return false;
            }

            std::string line;
            records_.clear();
            acs_.clear();
            atk_.clear();
            mach_.clear();

            // Skip header line.
            std::getline(file, line);

            while (std::getline(file, line)) {
                if (line.empty()) {
                    continue;
                }
                Record record{};
                if (!ParseLine(line, record)) {
                    continue;
                }
                records_.push_back(record);
                acs_.push_back(record.acs);
                atk_.push_back(record.atk);
                mach_.push_back(record.mach);
            }

            if (records_.empty()) {
                return false;
            }

            SortUnique(acs_);
            SortUnique(atk_);
            SortUnique(mach_);

            const size_t acsCount = acs_.size();
            const size_t atkCount = atk_.size();
            const size_t machCount = mach_.size();
            const size_t total = acsCount * atkCount * machCount;
            axial_.assign(total, std::numeric_limits<float>::quiet_NaN());
            normal_.assign(total, std::numeric_limits<float>::quiet_NaN());

            for (const Record &record : records_) {
                const size_t i = FindIndex(acs_, record.acs);
                const size_t j = FindIndex(atk_, record.atk);
                const size_t k = FindIndex(mach_, record.mach);
                const size_t idx = Flatten(i, j, k, atkCount, machCount);
                axial_[idx] = record.axial;
                normal_[idx] = record.normal;
            }

            loaded_ = true;
            return true;
        }

        /// \brief Check whether the table was loaded successfully.
        /// \return True if valid data is available.
        bool IsLoaded() const { return loaded_; }

        /// \brief Trilinearly interpolate axial and normal forces.
        /// \param acsDeg ACS angle in degrees.
        /// \param atkDeg Angle of attack in degrees.
        /// \param mach Mach number.
        /// \param axialOut Output axial force.
        /// \param normalOut Output normal force.
        void Interpolate(float acsDeg, float atkDeg, float mach, float &axialOut, float &normalOut) const {
            axialOut = 0.0f;
            normalOut = 0.0f;
            if (!loaded_) {
                return;
            }

            const size_t acsCount = acs_.size();
            const size_t atkCount = atk_.size();
            const size_t machCount = mach_.size();
            if (acsCount < 2 || atkCount < 2 || machCount < 2) {
                return;
            }

            const size_t i0 = FindLowerIndex(acs_, acsDeg);
            const size_t j0 = FindLowerIndex(atk_, atkDeg);
            const size_t k0 = FindLowerIndex(mach_, mach);
            const size_t i1 = std::min(i0 + 1, acsCount - 1);
            const size_t j1 = std::min(j0 + 1, atkCount - 1);
            const size_t k1 = std::min(k0 + 1, machCount - 1);

            const float tx = Fraction(acs_[i0], acs_[i1], acsDeg);
            const float ty = Fraction(atk_[j0], atk_[j1], atkDeg);
            const float tz = Fraction(mach_[k0], mach_[k1], mach);

            const float c000 = Get(axial_, i0, j0, k0, atkCount, machCount);
            const float c100 = Get(axial_, i1, j0, k0, atkCount, machCount);
            const float c010 = Get(axial_, i0, j1, k0, atkCount, machCount);
            const float c110 = Get(axial_, i1, j1, k0, atkCount, machCount);
            const float c001 = Get(axial_, i0, j0, k1, atkCount, machCount);
            const float c101 = Get(axial_, i1, j0, k1, atkCount, machCount);
            const float c011 = Get(axial_, i0, j1, k1, atkCount, machCount);
            const float c111 = Get(axial_, i1, j1, k1, atkCount, machCount);

            axialOut = Trilerp(c000, c100, c010, c110, c001, c101, c011, c111, tx, ty, tz);

            const float n000 = Get(normal_, i0, j0, k0, atkCount, machCount);
            const float n100 = Get(normal_, i1, j0, k0, atkCount, machCount);
            const float n010 = Get(normal_, i0, j1, k0, atkCount, machCount);
            const float n110 = Get(normal_, i1, j1, k0, atkCount, machCount);
            const float n001 = Get(normal_, i0, j0, k1, atkCount, machCount);
            const float n101 = Get(normal_, i1, j0, k1, atkCount, machCount);
            const float n011 = Get(normal_, i0, j1, k1, atkCount, machCount);
            const float n111 = Get(normal_, i1, j1, k1, atkCount, machCount);

            normalOut = Trilerp(n000, n100, n010, n110, n001, n101, n011, n111, tx, ty, tz);
        }

      private:
        /// \brief Parse one CSV line into a record.
        /// \param line CSV line text.
        /// \param record Output record.
        /// \return True on success.
        static bool ParseLine(const std::string &line, Record &record) {
            float values[5] = {};
            size_t index = 0;
            size_t start = 0;
            while (index < 5) {
                const size_t comma = line.find(',', start);
                const size_t end = (comma == std::string::npos) ? line.size() : comma;
                if (end <= start) {
                    return false;
                }
                const std::string token = line.substr(start, end - start);
                char *endPtr = nullptr;
                values[index] = std::strtof(token.c_str(), &endPtr);
                if (endPtr == token.c_str()) {
                    return false;
                }
                ++index;
                if (comma == std::string::npos) {
                    break;
                }
                start = comma + 1;
            }
            if (index < 5) {
                return false;
            }
            record.acs = values[0];
            record.atk = values[1];
            record.mach = values[2];
            record.axial = values[3];
            record.normal = values[4];
            return true;
        }

        /// \brief Sort a vector and remove duplicates.
        /// \param values Vector to modify.
        static void SortUnique(std::vector<float> &values) {
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end()), values.end());
        }

        /// \brief Find the nearest index for a target value.
        /// \param values Sorted lookup values.
        /// \param target Target value.
        /// \return Index of the nearest entry.
        static size_t FindIndex(const std::vector<float> &values, float target) {
            auto it = std::lower_bound(values.begin(), values.end(), target);
            if (it == values.end()) {
                return values.size() - 1;
            }
            return static_cast<size_t>(it - values.begin());
        }

        /// \brief Find the lower index for interpolation.
        /// \param values Sorted lookup values.
        /// \param target Target value.
        /// \return Lower index for interpolation.
        static size_t FindLowerIndex(const std::vector<float> &values, float target) {
            if (values.size() < 2) {
                return 0;
            }
            auto it = std::upper_bound(values.begin(), values.end(), target);
            if (it == values.begin()) {
                return 0;
            }
            if (it == values.end()) {
                return values.size() - 2;
            }
            return static_cast<size_t>((it - values.begin()) - 1);
        }

        /// \brief Compute interpolation fraction within a range.
        /// \param lower Lower bound.
        /// \param upper Upper bound.
        /// \param value Query value.
        /// \return Fraction between lower and upper.
        static float Fraction(float lower, float upper, float value) {
            const float denom = upper - lower;
            if (denom == 0.0f) {
                return 0.0f;
            }
            return (value - lower) / denom;
        }

        /// \brief Flatten 3D indices into a 1D array index.
        /// \param i ACS index.
        /// \param j ATK index.
        /// \param k Mach index.
        /// \param atkCount ATK dimension size.
        /// \param machCount Mach dimension size.
        /// \return Flattened index.
        static size_t Flatten(size_t i, size_t j, size_t k, size_t atkCount, size_t machCount) {
            return (i * atkCount + j) * machCount + k;
        }

        /// \brief Safely fetch a value from the flattened lookup.
        /// \param values Lookup array.
        /// \param i ACS index.
        /// \param j ATK index.
        /// \param k Mach index.
        /// \param atkCount ATK dimension size.
        /// \param machCount Mach dimension size.
        /// \return Value or 0.0f for missing data.
        static float Get(const std::vector<float> &values,
                         size_t i,
                         size_t j,
                         size_t k,
                         size_t atkCount,
                         size_t machCount) {
            const size_t idx = Flatten(i, j, k, atkCount, machCount);
            return values[idx];
        }

        /// \brief Linear interpolation between two values.
        /// \param a Start value.
        /// \param b End value.
        /// \param t Fraction between a and b.
        /// \return Interpolated value.
        static float Lerp(float a, float b, float t) {
            return a + (b - a) * t;
        }

        /// \brief Trilinear interpolation for a 3D grid.
        /// \param c000 Grid corner values.
        /// \param c100 Grid corner values.
        /// \param c010 Grid corner values.
        /// \param c110 Grid corner values.
        /// \param c001 Grid corner values.
        /// \param c101 Grid corner values.
        /// \param c011 Grid corner values.
        /// \param c111 Grid corner values.
        /// \param tx Fraction along x.
        /// \param ty Fraction along y.
        /// \param tz Fraction along z.
        /// \return Interpolated value.
        static float Trilerp(float c000, float c100, float c010, float c110,
                             float c001, float c101, float c011, float c111,
                             float tx, float ty, float tz) {
            const float x00 = Lerp(c000, c100, tx);
            const float x10 = Lerp(c010, c110, tx);
            const float x01 = Lerp(c001, c101, tx);
            const float x11 = Lerp(c011, c111, tx);
            const float y0 = Lerp(x00, x10, ty);
            const float y1 = Lerp(x01, x11, ty);
            return Lerp(y0, y1, tz);
        }

        /// \brief True once a table has been loaded.
        bool loaded_ = false;
        /// \brief Raw CFD records.
        std::vector<Record> records_;
        /// \brief Sorted unique ACS values.
        std::vector<float> acs_;
        /// \brief Sorted unique angle-of-attack values.
        std::vector<float> atk_;
        /// \brief Sorted unique Mach values.
        std::vector<float> mach_;
        /// \brief Axial force lookup grid.
        std::vector<float> axial_;
        /// \brief Normal force lookup grid.
        std::vector<float> normal_;
    };

    /// \brief Integrate one RK4 step.
    /// \param state Current state.
    /// \param dt Time step in seconds.
    /// \return Updated state.
    ApogeeState IntegrateStep(const ApogeeState &state, float dt) {
        const float halfDt = dt * 0.5f;
        const float sixthDt = dt * (1.0f / 6.0f);
        const Derivative k1 = Evaluate(state);
        const Derivative k2 = Evaluate(Apply(state, k1, halfDt));
        const Derivative k3 = Evaluate(Apply(state, k2, halfDt));
        const Derivative k4 = Evaluate(Apply(state, k3, dt));

        ApogeeState result = state;
        result.altitudeMeters += sixthDt * (k1.altitudeRate + 2.0f * k2.altitudeRate + 2.0f * k3.altitudeRate + k4.altitudeRate);
        result.horizontalDistanceMeters += sixthDt * (k1.horizontalRate + 2.0f * k2.horizontalRate + 2.0f * k3.horizontalRate + k4.horizontalRate);
        result.verticalVelocity += sixthDt * (k1.verticalAcceleration + 2.0f * k2.verticalAcceleration + 2.0f * k3.verticalAcceleration + k4.verticalAcceleration);
        result.horizontalVelocity += sixthDt * (k1.horizontalAcceleration + 2.0f * k2.horizontalAcceleration + 2.0f * k3.horizontalAcceleration + k4.horizontalAcceleration);
        result.zenith += sixthDt * (k1.zenithRate + 2.0f * k2.zenithRate + 2.0f * k3.zenithRate + k4.zenithRate);
        result.angularVelocity += sixthDt * (k1.angularAcceleration + 2.0f * k2.angularAcceleration + 2.0f * k3.angularAcceleration + k4.angularAcceleration);
        return result;
    }

    /// \brief Apply a derivative to a state.
    /// \param state Current state.
    /// \param derivative Derivative values.
    /// \param dt Time step in seconds.
    /// \return Updated state.
    ApogeeState Apply(const ApogeeState &state, const Derivative &derivative, float dt) const {
        ApogeeState result = state;
        result.altitudeMeters += derivative.altitudeRate * dt;
        result.horizontalDistanceMeters += derivative.horizontalRate * dt;
        result.verticalVelocity += derivative.verticalAcceleration * dt;
        result.horizontalVelocity += derivative.horizontalAcceleration * dt;
        result.zenith += derivative.zenithRate * dt;
        result.angularVelocity += derivative.angularAcceleration * dt;
        return result;
    }

    /// \brief Evaluate state derivatives for integration.
    /// \param state Current state.
    /// \return Derivative values.
    Derivative Evaluate(const ApogeeState &state) {
        const AccelResult accel = ComputeAcceleration(state);
        Derivative derivative;
        derivative.altitudeRate = state.verticalVelocity;
        derivative.horizontalRate = state.horizontalVelocity;
        derivative.verticalAcceleration = accel.linear.x;
        derivative.horizontalAcceleration = accel.linear.y;
        derivative.zenithRate = state.angularVelocity;
        derivative.angularAcceleration = accel.angular;
        return derivative;
    }

    /// \brief Compute linear and angular acceleration.
    /// \param state Current state.
    /// \return Acceleration results.
    AccelResult ComputeAcceleration(const ApogeeState &state) {
        const math_utils::Vec3 velocity = math_utils::MakeVec3(state.verticalVelocity, state.horizontalVelocity, 0.0f);
        math_utils::Vec3 velRelative = velocity;
        float temperature = 288.15f;
        if (environment_ != nullptr) {
            velRelative = math_utils::Subtract(velocity, environment_->GradientWind());
            temperature = environment_->TemperatureKelvin(state.altitudeMeters);
        }
        float speedOfSound = 0.0f;
        if (temperature > 0.0f) {
            speedOfSound = math_utils::FastSqrt(constants::kGamma * constants::kGasConstant * temperature);
        }
        float mach = 0.0f;
        if (speedOfSound > 0.0f) {
            mach = math_utils::Magnitude(velRelative) / speedOfSound;
        }

        const math_utils::Vec3 gravity = math_utils::MakeVec3(-constants::kGravity, 0.0f, 0.0f);
        math_utils::Vec3 linearAccel = gravity;
        float angularAccel = 0.0f;

        if (cfd_.IsLoaded() && mach >= 0.025f) {
            float velocityAngle = 0.0f;
            if (velRelative.x != 0.0f) {
                velocityAngle = atanf(velRelative.y / velRelative.x);
            } else if (velRelative.y != 0.0f) {
                velocityAngle = copysignf(1.5707963267948966f, velRelative.y);
            }
            velocityAngle = fabsf(velocityAngle);
            float attackAngle = state.zenith - velocityAngle;
            bool liftState = true;
            if (attackAngle < 0.0f) {
                liftState = false;
                attackAngle = -attackAngle;
            }
            const float attackAngleDeg = attackAngle * 57.29577951308232f;

            float axialForce = 0.0f;
            float normalForce = 0.0f;
            cfd_.Interpolate(vehicle_.acsAngleDeg, attackAngleDeg, mach, axialForce, normalForce);

            math_utils::Vec3 axialVec = math_utils::MakeVec3(-cosf(state.zenith), -sinf(state.zenith), 0.0f);
            math_utils::Vec3 normalVec = math_utils::MakeVec3(-sinf(state.zenith), cosf(state.zenith), 0.0f);

            math_utils::Vec3 axialForceVec = math_utils::Scale(axialVec, axialForce);
            math_utils::Vec3 normalForceVec = math_utils::Scale(normalVec, normalForce);

            float aeroMoment = -normalForce * vehicle_.centerOfPressureOffsetMeters;
            aeroMoment *= 0.2f;
            angularAccel = vehicle_.momentOfInertia > 0.0f ? aeroMoment / vehicle_.momentOfInertia : 0.0f;

            if (!liftState) {
                normalForceVec = math_utils::Scale(normalForceVec, -1.0f);
                angularAccel = -angularAccel;
            }

            math_utils::Vec3 aeroForce = math_utils::Add(axialForceVec, normalForceVec);
            if (vehicle_.dryMass > 0.0f) {
                const math_utils::Vec3 aeroAccel = math_utils::Scale(aeroForce, 1.0f / vehicle_.dryMass);
                linearAccel = math_utils::Add(gravity, aeroAccel);
            }
        }

        return AccelResult{linearAccel, angularAccel};
    }

    /// \brief Environment model pointer, optional.
    const EnvironmentModel *environment_ = nullptr;
    /// \brief Vehicle parameters for aero calculations.
    ApogeeVehicleParameters vehicle_;
    /// \brief CFD lookup table used for aerodynamic forces.
    CfdLookup cfd_;
    /// \brief Integration time step in seconds.
    float timeStep_ = 0.1f;
    /// \brief Minimum vertical velocity to attempt prediction.
    float minVerticalVelocityForPrediction_ = 0.0f;
};
