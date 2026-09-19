/*
 * Prints telemetry frames as hex, one per line, for a fixed set of inputs.
 *
 * test/test_firmware_parity.py compiles this, runs it, and checks the output
 * byte-for-byte against lora_bridge/protocol.py. That is the guard against the
 * classic failure of this kind of bridge: firmware and base station drifting
 * apart until every packet fails CRC on the far side of a radio link where you
 * cannot attach a debugger.
 *
 *     cc -o /tmp/parity firmware/test_parity.c -lm && /tmp/parity
 */

#include <stdio.h>
#include "rover_lora_tx/lora_packet.h"

struct sample {
    uint32_t seq;
    uint32_t rover_time_ms;
    double x, y, yaw, linear_velocity, angular_velocity;
};

int main(void)
{
    /* Chosen to cover: all-zero fields (many 0x00 bytes for COBS to stuff),
     * negatives, wrapped counters, and values past the fixed-point limits. */
    static const struct sample samples[] = {
        {0, 0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {7, 1234, 1.5, -2.25, 0.7853981634, 0.42, -0.1},
        {1, 100, -0.001, 0.001, -3.14159265, 0.0, 0.0},
        {4294967295u, 4294967295u, 1000.0, -1000.0, 3.14159265, 1.0, -1.0},
        {42, 65536, 0.0, 12.3456, 1.5707963, 1e6, -1e6},
        {255, 16777216, 2147.0, -2147.0, 0.0001, -0.001, 0.001},
    };
    const size_t count = sizeof(samples) / sizeof(samples[0]);

    for (size_t i = 0; i < count; i++) {
        uint8_t payload[LORA_TELEMETRY_SIZE];
        uint8_t frame[LORA_MAX_FRAME_SIZE];
        size_t payload_len = lora_pack_telemetry(
            samples[i].seq, samples[i].rover_time_ms,
            samples[i].x, samples[i].y, samples[i].yaw,
            samples[i].linear_velocity, samples[i].angular_velocity, payload);
        size_t frame_len = lora_encode_frame(
            LORA_MSG_ID_TELEMETRY, payload, payload_len, frame);

        if (frame_len == 0) {
            fprintf(stderr, "sample %zu failed to encode\n", i);
            return 1;
        }
        for (size_t b = 0; b < frame_len; b++) {
            printf("%02x", frame[b]);
        }
        printf("\n");
    }
    return 0;
}
