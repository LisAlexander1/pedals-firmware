#pragma once

#include <Arduino.h>

#ifndef CURVE_POINTS_N
#define CURVE_POINTS_N 6
#endif

constexpr uint8_t CURVE_POINTS = CURVE_POINTS_N;

enum AxisId : uint8_t {
    AXIS_THROTTLE = 0,
    AXIS_CLUTCH   = 1,
    AXIS_BRAKE    = 2,
    AXIS_COUNT    = 3,
};

enum FilterType : uint8_t {
    FILTER_NONE = 0,
    FILTER_EMA  = 1,
    FILTER_SMA  = 2,
};

struct CurvePoint {
    uint16_t x;
    uint16_t y;
};

struct AxisCalib {
    int32_t    rawMin;
    int32_t    rawMax;
    uint16_t   deadzoneLow;
    uint16_t   deadzoneHigh;
    bool       inverted;
    uint8_t    filterType;
    uint8_t    filterStrength;
    CurvePoint curve[CURVE_POINTS];
};

struct Calibration {
    uint32_t  magic;
    uint8_t   version;
    AxisCalib axes[AXIS_COUNT];
};

#ifndef CALIB_MAGIC_VAL
#define CALIB_MAGIC_VAL 0xC0FFEE02UL
#endif
#ifndef CALIB_VERSION_VAL
#define CALIB_VERSION_VAL 1
#endif

constexpr uint32_t CALIB_MAGIC   = CALIB_MAGIC_VAL;
constexpr uint8_t  CALIB_VERSION = CALIB_VERSION_VAL;

void calibration_load(Calibration& c);
void calibration_save(const Calibration& c);
void calibration_reset_axis(AxisCalib& a, AxisId id);
void calibration_reset_all(Calibration& c);
const char* axis_name(AxisId id);
bool axis_from_name(const char* name, AxisId& out);
