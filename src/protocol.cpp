#include "protocol.h"

#include <stdlib.h>
#include <string.h>

// ===========================================================================
// Текстовый строковый протокол. Никаких JSON-парсеров.
//
// Команды (PC → устройство), одна на строку, поля через пробел, \n завершает:
//   HI                                  — handshake, ждём ответ HELLO
//   STAT                                — разовый снимок состояния
//   CAL                                 — отдать всю калибровку
//   STR <0|1> [period_ms]               — стрим состояния вкл/выкл
//   SAV                                 — записать в EEPROM (принудительно)
//   RST [<axis>]                        — сброс одной оси либо всех
//   MIN <axis> [<value>]                — min: текущее raw, либо явно value
//   MAX <axis> [<value>]                — max: текущее raw, либо явно value
//   DZ  <axis> <low> <high>             — мёртвые зоны
//   INV <axis> <0|1>                    — инверсия
//   FLT <axis> <type:0..2> <strength>   — фильтр (0=none,1=ema,2=sma)
//   CRV <axis> x0 y0 x1 y1 x2 y2 x3 y3 x4 y4 x5 y5  — кривая
//   TAR <axis>                          — тара тензодатчика (только axis=2)
//
// Ответы (устройство → PC), одна на строку, \n завершает:
//   HELLO pedals <CALIB_VERSION>        — ответ на HI (магия "pedals")
//   OK <cmd>                            — успех
//   ER <cmd> <reason>                   — ошибка
//   S <ms> <r0> <r1> <r2> <o0> <o1> <o2> — снимок состояния
//   CAL                                 — начало блока калибровки
//   A <axis> <min> <max> <dzL> <dzH> <inv> <ftype> <fstr> <raw> <out>
//   C <axis> x0 y0 x1 y1 x2 y2 x3 y3 x4 y4 x5 y5
//   END                                 — конец блока калибровки
//
// axis: 0 = throttle, 1 = clutch, 2 = brake.
// ===========================================================================

#ifndef LINE_BUF_SIZE
#define LINE_BUF_SIZE 96
#endif

static char    lineBuf[LINE_BUF_SIZE];
static uint8_t lineLen = 0;

void protocol_init(ProtocolState& ps) {
    ps.streamEnabled  = false;
    ps.streamPeriodMs = STREAM_DEFAULT_PERIOD_MS;
    ps.lastStreamMs   = 0;
    lineLen = 0;
}

// ---- Простой токенайзер по пробелам/табуляции ----
static char* next_tok(char** sp) {
    char* s = *sp;
    if (!s) return nullptr;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == 0) { *sp = s; return nullptr; }
    char* start = s;
    while (*s != 0 && *s != ' ' && *s != '\t') s++;
    if (*s != 0) { *s = 0; s++; }
    *sp = s;
    return start;
}

static int t_int(char** sp, int def) {
    char* t = next_tok(sp);
    if (!t) return def;
    return atoi(t);
}

static bool resolve_axis_int(int v, AxisId& out) {
    if (v < 0 || v >= AXIS_COUNT) return false;
    out = (AxisId)v;
    return true;
}

// ---- Отправка ответов ----

static void send_ok(const char* cmd) {
    Serial.print(F("OK "));
    Serial.println(cmd);
}

static void send_err(const char* cmd, const char* msg) {
    Serial.print(F("ER "));
    Serial.print(cmd);
    Serial.print(' ');
    Serial.println(msg);
}

static void send_hello() {
    Serial.print(F("HELLO pedals "));
    Serial.println(CALIB_VERSION);
}

static void send_state(const AxisRuntime* rt) {
    Serial.print('S');
    Serial.print(' ');
    Serial.print(millis());
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(rt[i].rawCurrent);
    }
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(rt[i].outValue);
    }
    Serial.println();
}

static void send_axis_record(uint8_t i, const AxisCalib& a, const AxisRuntime& rt) {
    Serial.print(F("A "));
    Serial.print(i);
    Serial.print(' '); Serial.print(a.rawMin);
    Serial.print(' '); Serial.print(a.rawMax);
    Serial.print(' '); Serial.print(a.deadzoneLow);
    Serial.print(' '); Serial.print(a.deadzoneHigh);
    Serial.print(' '); Serial.print(a.inverted ? 1 : 0);
    Serial.print(' '); Serial.print(a.filterType);
    Serial.print(' '); Serial.print(a.filterStrength);
    Serial.print(' '); Serial.print(rt.rawCurrent);
    Serial.print(' '); Serial.print(rt.outValue);
    Serial.println();
}

static void send_curve_record(uint8_t i, const AxisCalib& a) {
    Serial.print(F("C "));
    Serial.print(i);
    for (uint8_t j = 0; j < CURVE_POINTS; ++j) {
        Serial.print(' '); Serial.print(a.curve[j].x);
        Serial.print(' '); Serial.print(a.curve[j].y);
    }
    Serial.println();
}

static void send_calibration(const Calibration& c, const AxisRuntime* rt) {
    Serial.println(F("CAL"));
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        send_axis_record(i, c.axes[i], rt[i]);
        send_curve_record(i, c.axes[i]);
    }
    Serial.println(F("END"));
}

// ---- Диспетчер команд ----

static void handle_cmd(char* cmd, char* rest, Calibration& calib,
                       const AxisRuntime* rt, ProtocolState& ps) {
    if (strcmp(cmd, "STAT") == 0) {
        send_state(rt);
        return;
    }
    if (strcmp(cmd, "CAL") == 0) {
        send_calibration(calib, rt);
        return;
    }
    if (strcmp(cmd, "HI") == 0) {
        send_hello();
        return;
    }
    if (strcmp(cmd, "STR") == 0) {
        int en = t_int(&rest, 0);
        ps.streamEnabled = (en != 0);
        char* p = next_tok(&rest);
        if (p) {
            int per = atoi(p);
            if (per < STREAM_MIN_PERIOD_MS) per = STREAM_MIN_PERIOD_MS;
            if (per > STREAM_MAX_PERIOD_MS) per = STREAM_MAX_PERIOD_MS;
            ps.streamPeriodMs = (uint16_t)per;
        }
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "SAV") == 0) {
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "RST") == 0) {
        char* a = next_tok(&rest);
        if (a) {
            int v = atoi(a); AxisId id;
            if (!resolve_axis_int(v, id)) { send_err(cmd, "axis"); return; }
            calibration_reset_axis(calib.axes[id], id);
        } else {
            calibration_reset_all(calib);
        }
        calibration_save(calib);
        send_ok(cmd);
        return;
    }

    // Все остальные команды требуют axis в первом аргументе.
    int axisInt = t_int(&rest, -1);
    AxisId axis;
    if (!resolve_axis_int(axisInt, axis)) { send_err(cmd, "axis"); return; }
    AxisCalib& a = calib.axes[axis];

    if (strcmp(cmd, "MIN") == 0) {
        // MIN <axis>           — взять текущее raw как min
        // MIN <axis> <value>   — задать min явно (для ручной правки из UI)
        char* val = next_tok(&rest);
        if (val && *val) {
            a.rawMin = atol(val);
        } else {
            a.rawMin = rt[axis].rawFiltered;
        }
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "MAX") == 0) {
        char* val = next_tok(&rest);
        if (val && *val) {
            a.rawMax = atol(val);
        } else {
            a.rawMax = rt[axis].rawFiltered;
        }
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "DZ") == 0) {
        int lo = t_int(&rest, 0);
        int hi = t_int(&rest, 0);
        if (lo < 0) lo = 0; if (lo > DEADZONE_MAX) lo = DEADZONE_MAX;
        if (hi < 0) hi = 0; if (hi > DEADZONE_MAX) hi = DEADZONE_MAX;
        a.deadzoneLow  = (uint16_t)lo;
        a.deadzoneHigh = (uint16_t)hi;
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "INV") == 0) {
        int v = t_int(&rest, 0);
        a.inverted = (v != 0);
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "FLT") == 0) {
        int t = t_int(&rest, 0);
        int s = t_int(&rest, 96);
        if (t < 0 || t > 2) { send_err(cmd, "type"); return; }
        if (s < 0) s = 0; if (s > 255) s = 255;
        a.filterType     = (uint8_t)t;
        a.filterStrength = (uint8_t)s;
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "CRV") == 0) {
        CurvePoint tmp[CURVE_POINTS];
        for (uint8_t i = 0; i < CURVE_POINTS; ++i) {
            int x = t_int(&rest, -1);
            int y = t_int(&rest, -1);
            if (x < AXIS_OUT_MIN || x > AXIS_OUT_MAX ||
                y < AXIS_OUT_MIN || y > AXIS_OUT_MAX) {
                send_err(cmd, "range");
                return;
            }
            tmp[i].x = (uint16_t)x;
            tmp[i].y = (uint16_t)y;
        }
        if (!pedals_curve_is_monotonic(tmp)) { send_err(cmd, "monotonic"); return; }
        memcpy(a.curve, tmp, sizeof(tmp));
        calibration_save(calib);
        send_ok(cmd);
        return;
    }
    if (strcmp(cmd, "TAR") == 0) {
        if (axis != AXIS_BRAKE) { send_err(cmd, "brake_only"); return; }
        a.rawMin = rt[axis].rawFiltered;
        calibration_save(calib);
        send_ok(cmd);
        return;
    }

    send_err(cmd, "unknown");
}

bool protocol_poll(ProtocolState& ps, Calibration& calib, const AxisRuntime* rt) {
    bool processed = false;
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                lineBuf[lineLen] = 0;
                char* rest = lineBuf;
                char* cmd  = next_tok(&rest);
                if (cmd && cmd[0]) {
                    handle_cmd(cmd, rest, calib, rt, ps);
                    processed = true;
                }
                lineLen = 0;
            }
        } else if (lineLen < LINE_BUF_SIZE - 1) {
            lineBuf[lineLen++] = (char)c;
        } else {
            lineLen = 0;
            send_err("?", "overflow");
        }
    }
    return processed;
}

void protocol_stream_tick(ProtocolState& ps, const AxisRuntime* rt) {
    if (!ps.streamEnabled) return;
    uint32_t now = millis();
    if (now - ps.lastStreamMs < ps.streamPeriodMs) return;
    ps.lastStreamMs = now;
    send_state(rt);
}
