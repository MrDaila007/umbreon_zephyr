/*
 * wifi_cipher.h — XOR stream cipher for UART config exchange.
 *
 * Included verbatim by both umbreon_zephyr (src/) and umbreon_esp_web (main/).
 * The PSK must be identical in both builds.
 *
 * Payload wire format for $WIFICFG: encrypt(plain) → hex string.
 * Payload format before encryption: "ssid\tpassword" (tab-separated, no NUL).
 * Maximum payload: 32 (SSID) + 1 (tab) + 63 (pass) = 96 bytes → 192 hex chars.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* Pre-shared key — change both firmwares together if rotating. */
static const uint8_t CFG_PSK[16] = {
    0xA3, 0x7F, 0x1C, 0xE8, 0x45, 0xB2, 0x9D, 0x03,
    0x6E, 0xF1, 0x28, 0xCC, 0x50, 0x87, 0xAB, 0x34
};

/* XOR stream cipher — same operation for encrypt and decrypt. */
static inline void cfg_xor(const uint8_t *in, uint8_t *out, size_t n)
{
    for (size_t i = 0; i < n; i++)
        out[i] = in[i] ^ CFG_PSK[i & 15u];
}

/* Encode `n` bytes to lowercase hex. out must hold 2*n+1 bytes.
 * Returns number of hex chars written (2*n), or -1 on overflow. */
static inline int cfg_to_hex(const uint8_t *data, size_t n,
                              char *out, size_t out_sz)
{
    static const char h[] = "0123456789abcdef";
    if (out_sz < n * 2u + 1u) return -1;
    for (size_t i = 0; i < n; i++) {
        out[i * 2u]      = h[data[i] >> 4];
        out[i * 2u + 1u] = h[data[i] & 0xfu];
    }
    out[n * 2u] = '\0';
    return (int)(n * 2u);
}

/* Decode hex string into at most `max` bytes.
 * Returns number of bytes decoded, or -1 on invalid input. */
static inline int cfg_from_hex(const char *s, uint8_t *out, size_t max)
{
    size_t i = 0;
    while (i < max && s[i * 2u] && s[i * 2u + 1u]) {
        char hi_c = s[i * 2u];
        char lo_c = s[i * 2u + 1u];
        uint8_t hi = (uint8_t)((hi_c >= 'a') ? hi_c - 'a' + 10 :
                               (hi_c >= 'A') ? hi_c - 'A' + 10 :
                                               hi_c - '0');
        uint8_t lo = (uint8_t)((lo_c >= 'a') ? lo_c - 'a' + 10 :
                               (lo_c >= 'A') ? lo_c - 'A' + 10 :
                                               lo_c - '0');
        out[i] = (uint8_t)((hi << 4) | lo);
        i++;
    }
    return (int)i;
}
