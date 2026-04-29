#include "calibration.h"

#include <EEPROM.h>
#include <string.h>

#ifndef CALIB_EEPROM_ADDR
#define CALIB_EEPROM_ADDR 0
#endif

static constexpr int EEPROM_BASE_ADDR = CALIB_EEPROM_ADDR;

static void make_default_curve(CurvePoint* curve) {
    for (uint8_t i = 0; i < CURVE_POINTS; ++i) {
        uint16_t v = (uint32_t)i * AXIS_OUT_MAX / (CURVE_POINTS - 1);
        curve[i].x = v;
        curve[i].y = v;
    }
}

void calibration_reset_axis(AxisCalib& a, AxisId id) {
    if (id == AXIS_BRAKE) {
        a.rawMin = DEFAULT_BRAKE_RAW_MIN;
        a.rawMax = DEFAULT_BRAKE_RAW_MAX;
    } else {
        a.rawMin = DEFAULT_HALL_RAW_MIN;
        a.rawMax = DEFAULT_HALL_RAW_MAX;
    }
    a.deadzoneLow    = DEFAULT_DEADZONE_LOW;
    a.deadzoneHigh   = DEFAULT_DEADZONE_HIGH;
    a.inverted       = false;
    a.filterType     = DEFAULT_FILTER_TYPE;
    a.filterStrength = DEFAULT_FILTER_STRENGTH;
    make_default_curve(a.curve);
}

void calibration_reset_all(Calibration& c) {
    c.magic   = CALIB_MAGIC;
    c.version = CALIB_VERSION;
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        calibration_reset_axis(c.axes[i], (AxisId)i);
    }
}

void calibration_load(Calibration& c) {
    EEPROM.get(EEPROM_BASE_ADDR, c);
    if (c.magic != CALIB_MAGIC || c.version != CALIB_VERSION) {
        calibration_reset_all(c);
        calibration_save(c);
    }
}

void calibration_save(const Calibration& c) {
    EEPROM.put(EEPROM_BASE_ADDR, c);
}

const char* axis_name(AxisId id) {
    switch (id) {
        case AXIS_THROTTLE: return "throttle";
        case AXIS_CLUTCH:   return "clutch";
        case AXIS_BRAKE:    return "brake";
        default:            return "?";
    }
}

bool axis_from_name(const char* name, AxisId& out) {
    if (!name) return false;
    if (strcmp(name, "throttle") == 0) { out = AXIS_THROTTLE; return true; }
    if (strcmp(name, "clutch")   == 0) { out = AXIS_CLUTCH;   return true; }
    if (strcmp(name, "brake")    == 0) { out = AXIS_BRAKE;    return true; }
    return false;
}
