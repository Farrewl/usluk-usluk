/*
 * core/src/ecu_link.c — Implementasi protokol UART ECU (C murni)
 */

#include "ecu_link.h"

int ecu_parse_frame(const unsigned char *raw, ecu_status_t *out) {
    unsigned char sum;
    int valid = 0;

    if (raw == 0 || out == 0) return 0;

    out->valid = 0;
    out->kill = 0;
    out->estop = 0;
    out->overtemp = 0;
    out->overcurrent = 0;
    out->current_a = 0.0;
    out->temp_esc_c = 0.0;
    out->temp_amb_c = 0.0;
    out->voltage_v = 0.0;

    if (raw[0] != ECU_HDR_0 || raw[1] != ECU_HDR_1) return 0;

    sum = (unsigned char)((raw[2] + raw[3] + raw[4] + raw[5] + raw[6]) & 0xFF);
    if (sum != raw[7]) return 0;

    valid = 1;
    out->valid = 1;
    out->kill        = (raw[2] & ECU_FLAG_KILL)        ? 1 : 0;
    out->estop       = (raw[2] & ECU_FLAG_ESTOP)       ? 1 : 0;
    out->overtemp    = (raw[2] & ECU_FLAG_OVERTEMP)    ? 1 : 0;
    out->overcurrent = (raw[2] & ECU_FLAG_OVERCURRENT) ? 1 : 0;
    out->current_a = (double)raw[3] / 10.0;
    out->temp_esc_c = (double)raw[4];
    out->temp_amb_c = (double)raw[5];
    out->voltage_v = (double)raw[6] / 10.0;
    return valid;
}

unsigned char ecu_build_cmd(int cmd) {
    if (cmd == ECU_CMD_KILL)  return ECU_CMD_KILL;
    if (cmd == ECU_CMD_CLEAR) return ECU_CMD_CLEAR;
    return ECU_CMD_NORMAL;
}
