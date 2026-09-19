/*
 * lora_packet.h - rover-side wire protocol for the diff_drive_robot LoRa link.
 *
 * Header-only C translation of lora_bridge/protocol.py. The two MUST stay in
 * step: if you change a field here, change it there, and re-run
 * firmware/test_parity.c to prove the byte streams still match.
 *
 * Frame layout:
 *     COBS( msg_id | len_hi | len_lo | crc_hi | crc_lo | payload ) 0x00
 *
 * Header fields are big endian; payload fields are little endian.
 */

#ifndef LORA_PACKET_H
#define LORA_PACKET_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#define LORA_MSG_ID_TELEMETRY 0x01
#define LORA_HEADER_SIZE 5
#define LORA_TELEMETRY_SIZE 22
/* Worst case: header + payload + COBS overhead (1 per 254 bytes) + delimiter. */
#define LORA_MAX_FRAME_SIZE 64

/* Fixed-point scales. Identical literals to protocol.py. */
#define LORA_MM_PER_M 1000.0
#define LORA_CDEG_PER_RAD 5729.578
#define LORA_MMS_PER_MS 1000.0
#define LORA_MRAD_PER_RAD 1000.0

static inline int32_t lora_clamp_i32(double v)
{
    if (v > 2147483647.0) return (int32_t)2147483647;
    if (v < -2147483648.0) return (int32_t)(-2147483647 - 1);
    return (int32_t)lround(v);
}

static inline int16_t lora_clamp_i16(double v)
{
    if (v > 32767.0) return (int16_t)32767;
    if (v < -32768.0) return (int16_t)-32768;
    return (int16_t)lround(v);
}

/* Explicit little-endian writes: never memcpy a struct, the base station must
 * not depend on this MCU's byte order or padding rules. */
static inline void lora_put_u32le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
    dst[2] = (uint8_t)((value >> 16) & 0xFF);
    dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

static inline void lora_put_u16le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout. */
static inline uint16_t lora_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* COBS-encode len bytes of src into dst. dst needs len + len/254 + 2 bytes.
 * Returns the number of bytes written; the output contains no 0x00. */
static inline size_t lora_cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < len) {
        if (src[read_index] == 0) {
            dst[code_index] = code;
            code_index = write_index++;
            code = 1;
        } else {
            dst[write_index++] = src[read_index];
            code++;
            if (code == 0xFF) {
                dst[code_index] = code;
                code_index = write_index++;
                code = 1;
            }
        }
        read_index++;
    }
    dst[code_index] = code;
    return write_index;
}

/* Pack rover pose and velocity into the 22-byte telemetry payload.
 * Units in: metres, radians, m/s, rad/s. Returns bytes written. */
static inline size_t lora_pack_telemetry(uint32_t seq, uint32_t rover_time_ms,
                                         double x, double y, double yaw,
                                         double linear_velocity,
                                         double angular_velocity,
                                         uint8_t *out)
{
    lora_put_u32le(out + 0, seq);
    lora_put_u32le(out + 4, rover_time_ms);
    lora_put_u32le(out + 8, (uint32_t)lora_clamp_i32(x * LORA_MM_PER_M));
    lora_put_u32le(out + 12, (uint32_t)lora_clamp_i32(y * LORA_MM_PER_M));
    lora_put_u16le(out + 16, (uint16_t)lora_clamp_i16(yaw * LORA_CDEG_PER_RAD));
    lora_put_u16le(out + 18, (uint16_t)lora_clamp_i16(linear_velocity * LORA_MMS_PER_MS));
    lora_put_u16le(out + 20, (uint16_t)lora_clamp_i16(angular_velocity * LORA_MRAD_PER_RAD));
    return LORA_TELEMETRY_SIZE;
}

/* Build a complete on-the-wire frame, trailing 0x00 delimiter included.
 * Returns bytes written to out, or 0 if the payload does not fit. */
static inline size_t lora_encode_frame(uint8_t msg_id, const uint8_t *payload,
                                       size_t payload_len, uint8_t *out)
{
    uint8_t raw[LORA_HEADER_SIZE + LORA_TELEMETRY_SIZE];
    uint16_t crc;
    size_t encoded;

    if (msg_id == 0 || payload_len > sizeof(raw) - LORA_HEADER_SIZE) {
        return 0;  /* msg_id 0 is the COBS delimiter and can never be valid */
    }

    crc = lora_crc16_ccitt(payload, payload_len);
    raw[0] = msg_id;
    raw[1] = (uint8_t)((payload_len >> 8) & 0xFF);
    raw[2] = (uint8_t)(payload_len & 0xFF);
    raw[3] = (uint8_t)((crc >> 8) & 0xFF);
    raw[4] = (uint8_t)(crc & 0xFF);
    for (size_t i = 0; i < payload_len; i++) {
        raw[LORA_HEADER_SIZE + i] = payload[i];
    }

    encoded = lora_cobs_encode(raw, LORA_HEADER_SIZE + payload_len, out);
    out[encoded] = 0x00;
    return encoded + 1;
}

#endif /* LORA_PACKET_H */
