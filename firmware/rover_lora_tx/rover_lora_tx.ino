/*
 * rover_lora_tx - ESP32 telemetry transmitter for diff_drive_robot.
 *
 *   sensors -> ESP32 -> UART -> LoRa module ~~~ RF ~~~> base station
 *
 * The ESP32 talks to its LoRa module over a local UART only; the radio link
 * itself is the wireless hop. Nothing here knows about ROS 2 -- the base
 * station turns these frames into topics.
 *
 * Wiring (ESP32 DevKit v1, UART2):
 *
 *   ESP32 GPIO16 (RX2) <- LoRa TXD
 *   ESP32 GPIO17 (TX2) -> LoRa RXD
 *   ESP32 GND          -- LoRa GND
 *   ESP32 3V3          -- LoRa VCC   (check your module: E22 wants 3.3 V and
 *                                     can draw >100 mA while transmitting,
 *                                     which is more than some USB-serial
 *                                     adapters supply. Use a proper 3.3 V
 *                                     rail with a bulk capacitor.)
 *
 * Ebyte E22/E32 modules also have M0/M1 mode pins. Both LOW selects normal
 * transparent mode, which is what this sketch assumes: whatever bytes go in
 * one module's UART come out of the other module's UART. Tie them low in
 * hardware, or drive them with the optional pins below.
 *
 * Both radios must share the same channel, address, air data rate and
 * transmit power, or nothing will be received. Configure them once with the
 * vendor tool before running this.
 */

#include "lora_packet.h"

/* --- local UART to the LoRa module ------------------------------------- */
static const int LORA_RX_PIN = 16;
static const int LORA_TX_PIN = 17;
static const uint32_t LORA_UART_BAUD = 9600;  /* must match the module config */

/* Optional Ebyte mode pins. Set to -1 if you tied M0/M1 low in hardware. */
static const int LORA_M0_PIN = -1;
static const int LORA_M1_PIN = -1;

/* --- telemetry rate ------------------------------------------------------
 * 2 Hz of 29-byte frames is ~58 B/s, comfortably inside what LoRa sustains at
 * SF7-SF9 and inside the 1% duty-cycle limits that apply in some regions.
 * Raising this much above 5 Hz will start dropping packets, and in the EU it
 * will also breach the duty cycle. Slower is better for range.              */
static const uint32_t TELEMETRY_PERIOD_MS = 500;

static uint32_t g_seq = 0;
static uint32_t g_last_send_ms = 0;

/* --- rover state ---------------------------------------------------------
 * Replace the body of this function with your real sensor reads. It currently
 * integrates a simple differential-drive model so the link can be verified
 * end to end the moment the sketch is flashed, before any encoder or IMU is
 * wired up. The caller expects metres, radians, m/s and rad/s.
 *
 * TODO(hardware): read wheel encoder ticks, convert to wheel velocities with
 * the wheel radius and tick count, combine into body velocity with the wheel
 * separation, and integrate for pose. Fuse yaw with an IMU if one is fitted.
 */
static void readRoverState(double dt, double *x, double *y, double *yaw,
                           double *linear_velocity, double *angular_velocity)
{
    static double sim_x = 0.0;
    static double sim_y = 0.0;
    static double sim_yaw = 0.0;
    static double sim_elapsed = 0.0;

    const double linear_speed = 0.25;       /* m/s   */
    const double angular_amplitude = 0.4;   /* rad/s */
    const double angular_period = 20.0;     /* s     */

    sim_elapsed += dt;
    double w = angular_amplitude * sin(2.0 * M_PI * sim_elapsed / angular_period);

    sim_yaw += w * dt;
    sim_yaw = atan2(sin(sim_yaw), cos(sim_yaw));  /* wrap to +-pi */
    sim_x += linear_speed * cos(sim_yaw) * dt;
    sim_y += linear_speed * sin(sim_yaw) * dt;

    *x = sim_x;
    *y = sim_y;
    *yaw = sim_yaw;
    *linear_velocity = linear_speed;
    *angular_velocity = w;
}

void setup()
{
    Serial.begin(115200);  /* USB debug console */

    if (LORA_M0_PIN >= 0) {
        pinMode(LORA_M0_PIN, OUTPUT);
        digitalWrite(LORA_M0_PIN, LOW);
    }
    if (LORA_M1_PIN >= 0) {
        pinMode(LORA_M1_PIN, OUTPUT);
        digitalWrite(LORA_M1_PIN, LOW);
    }

    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

    delay(100);  /* let the module settle after power-up */
    g_last_send_ms = millis();
    Serial.println(F("rover_lora_tx ready"));
}

void loop()
{
    uint32_t now = millis();

    /* Unsigned subtraction, so this stays correct across the ~49 day millis()
     * rollover instead of stalling the rover for a month. */
    if ((uint32_t)(now - g_last_send_ms) < TELEMETRY_PERIOD_MS) {
        return;
    }
    double dt = (double)(uint32_t)(now - g_last_send_ms) / 1000.0;
    g_last_send_ms = now;

    double x, y, yaw, linear_velocity, angular_velocity;
    readRoverState(dt, &x, &y, &yaw, &linear_velocity, &angular_velocity);

    uint8_t payload[LORA_TELEMETRY_SIZE];
    uint8_t frame[LORA_MAX_FRAME_SIZE];

    size_t payload_len = lora_pack_telemetry(g_seq, now, x, y, yaw,
                                             linear_velocity, angular_velocity,
                                             payload);
    size_t frame_len = lora_encode_frame(LORA_MSG_ID_TELEMETRY, payload,
                                         payload_len, frame);
    if (frame_len == 0) {
        Serial.println(F("encode failed"));
        return;
    }

    Serial2.write(frame, frame_len);
    g_seq++;

    Serial.printf("seq %lu  x=%.2f y=%.2f yaw=%.3f  (%u bytes)\n",
                  (unsigned long)g_seq, x, y, yaw, (unsigned)frame_len);
}
