#pragma once

#include <stdint.h>
#include <string.h>

// Fixed companion telemetry cache. Arrival of one field never renews another.
struct USVPayload {
    static constexpr uint8_t FIELD_COUNT = 26;
    static constexpr uint32_t MAX_AGE_MS = 3000;

    static const char *name(uint8_t index)
    {
        static const char names[FIELD_COUNT][11] = {
            "USV_VOLT", "USV_ABS", "PUMP_X", "PUMP_Y", "PUMP_Z", "PUMP_A",
            "USV_STAT", "USV_PKT", "USV_STEP", "USV_STOT", "USV_LOOP", "USV_LTOT",
            "USV_SCNT", "USV_PERR", "USV_PMOD", "USV_BSET", "USV_REF", "USV_BASE",
            "USV_VLD", "USV_JTMP", "USV_ETMP", "USV_JCPU", "USV_JMEM", "USV_EHEAP",
            "USV_ADS", "USV_PROTO"
        };
        return names[index];
    }

    bool update(const char *field_name, float value, uint32_t now_ms)
    {
        for (uint8_t i = 0; i < FIELD_COUNT; i++) {
            if (strcmp(field_name, name(i)) == 0) {
                values[i] = value;
                received_ms[i] = now_ms;
                seen |= uint32_t(1) << i;
                last_update_ms = now_ms;
                return true;
            }
        }
        return false;
    }

    bool fresh(uint8_t index, uint32_t now_ms) const
    {
        return (seen & (uint32_t(1) << index)) != 0 &&
               uint32_t(now_ms - received_ms[index]) <= MAX_AGE_MS;
    }

    float values[FIELD_COUNT] {};
    uint32_t received_ms[FIELD_COUNT] {};
    uint32_t seen = 0;
    uint32_t last_update_ms = 0; // Retained for the mission companion watchdog only.
};
