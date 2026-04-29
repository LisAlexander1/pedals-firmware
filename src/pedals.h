#pragma once

#include <Arduino.h>
#include "calibration.h"

#ifndef SMA_MAX_WINDOW_N
#define SMA_MAX_WINDOW_N 32
#endif

constexpr uint8_t SMA_MAX_WINDOW = SMA_MAX_WINDOW_N;

struct AxisRuntime {
    int32_t  rawCurrent;     // последнее сырое (до фильтра)
    int32_t  rawFiltered;    // после фильтра
    uint16_t outValue;       // 0..1023, что уходит в HID
    int32_t  emaState;       // состояние EMA (Q16: значение << 8)
    int16_t  smaBuf[SMA_MAX_WINDOW];
    uint8_t  smaIdx;
    uint8_t  smaCount;
};

void pedals_init(AxisRuntime* runtime, uint8_t count);
int32_t pedals_filter(int32_t raw, const AxisCalib& cfg, AxisRuntime& rt);
uint16_t pedals_process(int32_t raw_filtered, const AxisCalib& cfg);
void pedals_apply_curve_check(CurvePoint* curve);
bool pedals_curve_is_monotonic(const CurvePoint* curve);
