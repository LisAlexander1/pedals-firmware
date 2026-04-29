#include <Arduino.h>
#include <HX711.h>
#include <Joystick.h>

#include "calibration.h"
#include "pedals.h"
#include "protocol.h"

static Calibration   g_calib;
static AxisRuntime   g_runtime[AXIS_COUNT];
static ProtocolState g_proto;
static HX711         g_scale;
static int32_t       g_lastBrakeRaw = 0;

static Joystick_ Joystick(
    JOYSTICK_DEFAULT_REPORT_ID,
    JOYSTICK_TYPE_JOYSTICK,
    0, 0,                 // 0 кнопок, 0 hatswitch
    false, false, false,  // X, Y, Z
    false, false, false,  // Rx, Ry, Rz
    false, false,         // Rudder, Throttle
    true,  true,  true    // Accelerator, Brake, Steering
);

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.setTimeout(SERIAL_TIMEOUT_MS);

    calibration_load(g_calib);
    pedals_init(g_runtime, AXIS_COUNT);
    protocol_init(g_proto);

    g_scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
    g_scale.set_gain(HX711_GAIN);

    Joystick.begin(false);  // false = ручной sendState()
    Joystick.setAcceleratorRange(AXIS_OUT_MIN, AXIS_OUT_MAX);
    Joystick.setBrakeRange(AXIS_OUT_MIN, AXIS_OUT_MAX);
    Joystick.setSteeringRange(AXIS_OUT_MIN, AXIS_OUT_MAX);
}

void loop() {
    // 1. Сырое чтение
    if (g_scale.is_ready()) {
        g_lastBrakeRaw = g_scale.read();
    }
    int32_t raw_throttle = analogRead(PIN_THROTTLE);
    int32_t raw_clutch   = analogRead(PIN_CLUTCH);
    int32_t raw_brake    = g_lastBrakeRaw;

    // 2. Фильтр
    int32_t f_throttle = pedals_filter(raw_throttle, g_calib.axes[AXIS_THROTTLE], g_runtime[AXIS_THROTTLE]);
    int32_t f_clutch   = pedals_filter(raw_clutch,   g_calib.axes[AXIS_CLUTCH],   g_runtime[AXIS_CLUTCH]);
    int32_t f_brake    = pedals_filter(raw_brake,    g_calib.axes[AXIS_BRAKE],    g_runtime[AXIS_BRAKE]);

    // 3. Нормализация + deadzone + кривая
    uint16_t out_throttle = pedals_process(f_throttle, g_calib.axes[AXIS_THROTTLE]);
    uint16_t out_clutch   = pedals_process(f_clutch,   g_calib.axes[AXIS_CLUTCH]);
    uint16_t out_brake    = pedals_process(f_brake,    g_calib.axes[AXIS_BRAKE]);

    g_runtime[AXIS_THROTTLE].outValue = out_throttle;
    g_runtime[AXIS_CLUTCH].outValue   = out_clutch;
    g_runtime[AXIS_BRAKE].outValue    = out_brake;

    // 4. HID
    Joystick.setAccelerator(out_throttle);
    Joystick.setSteering(out_clutch);
    Joystick.setBrake(out_brake);
    Joystick.sendState();

    // 5. Serial
    protocol_poll(g_proto, g_calib, g_runtime);
    protocol_stream_tick(g_proto, g_runtime);
}
