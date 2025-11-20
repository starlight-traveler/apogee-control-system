#pragma once

#include <cmath>

#include "constants.h"
#include "environment_model.h"
#include "math_utils.h"

// C++ equivalent to apogee.py and apogee_lib.py focused on ballistic prediction.

struct ApogeeVehicleParameters {
    float centerOfPressureOffsetMeters = 0.0f;  // cp_cg in Python (m)
    float momentOfInertia = 1.0f;               // kg·m^2
    float dryMass = 1.0f;                       // kg
};

struct ApogeeState {
    float altitudeMeters = 0.0f;
    float horizontalDistanceMeters = 0.0f;
    float verticalVelocity = 0.0f;
    float horizontalVelocity = 0.0f;
    float zenith = 0.0f;
    float angularVelocity = 0.0f;
};

class ApogeePredictor {
  public:
    ApogeePredictor(const EnvironmentModel &environment, const ApogeeVehicleParameters &vehicle)
        : environment_(&environment), vehicle_(vehicle) {}

    ApogeePredictor() = default;

    void SetEnvironment(const EnvironmentModel &environment) { environment_ = &environment; }
    void SetVehicleParameters(const ApogeeVehicleParameters &vehicle) { vehicle_ = vehicle; }

    void SetTimeStep(float dt) { timeStep_ = dt; }

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
    struct Derivative {
        float altitudeRate;
        float horizontalRate;
        float verticalAcceleration;
        float horizontalAcceleration;
        float zenithRate;
        float angularAcceleration;
    };

    struct AccelResult {
        math_utils::Vec3 linear;  // x: vertical, y: horizontal, z: unused
        float angular;
    };

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

        (void)vehicle_;
        (void)mach;  // Placeholder until CFD lookup tables are integrated onboard.

        const math_utils::Vec3 gravity = math_utils::MakeVec3(-constants::kGravity, 0.0f, 0.0f);
        const math_utils::Vec3 aeroForce = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
        math_utils::Vec3 linearAccel = aeroForce;
        linearAccel = math_utils::Add(linearAccel, gravity);

        const float angularAccel = 0.0f;

        return AccelResult{linearAccel, angularAccel};
    }

    const EnvironmentModel *environment_ = nullptr;
    ApogeeVehicleParameters vehicle_;
    float timeStep_ = 0.1f;
    float minVerticalVelocityForPrediction_ = 1.0f;
};
