#pragma once

#include <Arduino.h>
#include "calibration.h"
#include "pedals.h"

struct ProtocolState {
    bool     streamEnabled;
    uint16_t streamPeriodMs;
    uint32_t lastStreamMs;
};

void protocol_init(ProtocolState& ps);
// Обработать одну входящую JSON-строку (если доступна). Возвращает true если что-то обработали.
bool protocol_poll(ProtocolState& ps, Calibration& calib, const AxisRuntime* rt);
// Отправить состояние (raw + out) если включён стрим и подошло время.
void protocol_stream_tick(ProtocolState& ps, const AxisRuntime* rt);
