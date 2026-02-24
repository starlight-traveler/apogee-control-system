#include "status_leds.h"

#include <WiFiNINA.h>

#include "settings.h"

namespace {

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

bool g_faultActive = false;
bool g_wifiConnected = false;
bool g_subscriberActive = false;
bool g_manualOverride = false;
FlightStatus g_flightStatus = FlightStatus::Ground;

uint32_t g_lastUpdateMs = 0;
bool g_blinkPhaseOn = false;
Rgb g_lastApplied{0, 0, 0};

Rgb ColorForFlightStatus(FlightStatus status) {
    switch (status) {
        case FlightStatus::Ground:
            return {settings::status_leds::kGroundR,
                    settings::status_leds::kGroundG,
                    settings::status_leds::kGroundB};
        case FlightStatus::Burn:
            return {settings::status_leds::kBurnR, settings::status_leds::kBurnG, settings::status_leds::kBurnB};
        case FlightStatus::Coast:
            return {settings::status_leds::kCoastR,
                    settings::status_leds::kCoastG,
                    settings::status_leds::kCoastB};
        case FlightStatus::Overshoot:
            return {settings::status_leds::kOvershootR,
                    settings::status_leds::kOvershootG,
                    settings::status_leds::kOvershootB};
        case FlightStatus::Descent:
            return {settings::status_leds::kDescentR,
                    settings::status_leds::kDescentG,
                    settings::status_leds::kDescentB};
    }
    return {0, 0, 0};
}

Rgb ComputeTargetColor() {
    if (g_faultActive) {
        return g_blinkPhaseOn ? Rgb{settings::status_leds::kFaultR,
                                    settings::status_leds::kFaultG,
                                    settings::status_leds::kFaultB}
                              : Rgb{0, 0, 0};
    }
    if (!g_wifiConnected) {
        return g_blinkPhaseOn ? Rgb{settings::status_leds::kWifiDownR,
                                    settings::status_leds::kWifiDownG,
                                    settings::status_leds::kWifiDownB}
                              : Rgb{0, 0, 0};
    }
    if (g_manualOverride) {
        return g_blinkPhaseOn ? Rgb{settings::status_leds::kManualR,
                                    settings::status_leds::kManualG,
                                    settings::status_leds::kManualB}
                              : Rgb{0, 0, 0};
    }
    if (!g_subscriberActive) {
        return g_blinkPhaseOn ? Rgb{settings::status_leds::kNoSubscriberR,
                                    settings::status_leds::kNoSubscriberG,
                                    settings::status_leds::kNoSubscriberB}
                              : Rgb{0, 0, 0};
    }
    return ColorForFlightStatus(g_flightStatus);
}

}  // namespace

void StatusLedsBegin() {
    g_lastUpdateMs = millis();
    g_blinkPhaseOn = false;
    g_lastApplied = {0, 0, 0};
    WiFi.setLEDs(0, 0, 0);
}

void StatusLedsSetFault(bool active) { g_faultActive = active; }

void StatusLedsSetFlightStatus(FlightStatus status) { g_flightStatus = status; }

void StatusLedsSetComms(bool wifiConnected, bool subscriberActive) {
    g_wifiConnected = wifiConnected;
    g_subscriberActive = subscriberActive;
}

void StatusLedsSetManualOverride(bool active) { g_manualOverride = active; }

void StatusLedsService(uint32_t nowMs) {
    if ((nowMs - g_lastUpdateMs) < settings::status_leds::kUpdateIntervalMs) {
        return;
    }
    g_lastUpdateMs = nowMs;
    g_blinkPhaseOn = !g_blinkPhaseOn;

    const Rgb target = ComputeTargetColor();
    if (target.r == g_lastApplied.r && target.g == g_lastApplied.g && target.b == g_lastApplied.b) {
        return;
    }

    WiFi.setLEDs(target.r, target.g, target.b);
    g_lastApplied = target;
}
