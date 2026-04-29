#include "pedals.h"

#include <string.h>

void pedals_init(AxisRuntime* runtime, uint8_t count) {
    memset(runtime, 0, sizeof(AxisRuntime) * count);
}

static int32_t apply_ema(int32_t raw, const AxisCalib& cfg, AxisRuntime& rt) {
    // y[n] = y[n-1] + alpha * (x[n] - y[n-1]); alpha = strength/255
    // Используем целочисленное вычисление с фиксированной точкой Q8 на множителе.
    if (rt.smaCount == 0 && rt.emaState == 0) {
        rt.emaState = (int32_t)raw << 8;
    }
    int32_t y_prev = rt.emaState >> 8;
    int32_t diff = raw - y_prev;
    int32_t step = (diff * (int32_t)cfg.filterStrength) / 255;
    rt.emaState += step << 8;
    return rt.emaState >> 8;
}

static int32_t apply_sma(int32_t raw, const AxisCalib& cfg, AxisRuntime& rt) {
    uint8_t window = cfg.filterStrength;
    if (window < 1) window = 1;
    if (window > SMA_MAX_WINDOW) window = SMA_MAX_WINDOW;

    rt.smaBuf[rt.smaIdx] = (int16_t)constrain(raw, INT16_MIN, INT16_MAX);
    rt.smaIdx = (rt.smaIdx + 1) % window;
    if (rt.smaCount < window) rt.smaCount++;

    int32_t sum = 0;
    for (uint8_t i = 0; i < rt.smaCount; ++i) {
        sum += rt.smaBuf[i];
    }
    return sum / rt.smaCount;
}

int32_t pedals_filter(int32_t raw, const AxisCalib& cfg, AxisRuntime& rt) {
    rt.rawCurrent = raw;
    int32_t out;
    switch (cfg.filterType) {
        case FILTER_EMA: out = apply_ema(raw, cfg, rt); break;
        case FILTER_SMA: out = apply_sma(raw, cfg, rt); break;
        default:         out = raw; break;
    }
    rt.rawFiltered = out;
    return out;
}

static uint16_t apply_curve(uint16_t x, const CurvePoint* curve) {
    if (x <= curve[0].x) return curve[0].y;
    if (x >= curve[CURVE_POINTS - 1].x) return curve[CURVE_POINTS - 1].y;
    for (uint8_t i = 0; i < CURVE_POINTS - 1; ++i) {
        if (x >= curve[i].x && x <= curve[i + 1].x) {
            uint32_t dx = (uint32_t)curve[i + 1].x - curve[i].x;
            if (dx == 0) return curve[i].y;
            int32_t dy = (int32_t)curve[i + 1].y - (int32_t)curve[i].y;
            int32_t v = (int32_t)curve[i].y + (dy * (int32_t)(x - curve[i].x)) / (int32_t)dx;
            if (v < 0) v = 0;
            if (v > AXIS_OUT_MAX) v = AXIS_OUT_MAX;
            return (uint16_t)v;
        }
    }
    return curve[CURVE_POINTS - 1].y;
}

uint16_t pedals_process(int32_t raw_filtered, const AxisCalib& cfg) {
    int32_t lo = cfg.rawMin;
    int32_t hi = cfg.rawMax;
    if (cfg.inverted) {
        int32_t t = lo; lo = hi; hi = t;
    }

    int32_t span = hi - lo;
    int32_t v;
    if (span == 0) {
        v = 0;
    } else if (span > 0) {
        if (raw_filtered <= lo) v = 0;
        else if (raw_filtered >= hi) v = AXIS_OUT_MAX;
        else v = ((int64_t)(raw_filtered - lo) * AXIS_OUT_MAX) / span;
    } else {
        // hi < lo -> инверсия через перестановку: пересчитываем в обратную сторону
        if (raw_filtered >= lo) v = 0;
        else if (raw_filtered <= hi) v = AXIS_OUT_MAX;
        else v = ((int64_t)(lo - raw_filtered) * AXIS_OUT_MAX) / (-span);
    }

    // Мёртвые зоны
    int32_t dzL = cfg.deadzoneLow;
    int32_t dzH = cfg.deadzoneHigh;
    if (dzL + dzH >= AXIS_OUT_MAX) { dzL = 0; dzH = 0; }
    int32_t inner_lo = dzL;
    int32_t inner_hi = AXIS_OUT_MAX - dzH;
    int32_t inner_span = inner_hi - inner_lo;
    if (v <= inner_lo) v = 0;
    else if (v >= inner_hi) v = AXIS_OUT_MAX;
    else v = ((int64_t)(v - inner_lo) * AXIS_OUT_MAX) / inner_span;

    // Кривая
    uint16_t curved = apply_curve((uint16_t)v, cfg.curve);
    return curved;
}

bool pedals_curve_is_monotonic(const CurvePoint* curve) {
    for (uint8_t i = 0; i + 1 < CURVE_POINTS; ++i) {
        if (curve[i + 1].x <= curve[i].x) return false;
    }
    return true;
}
