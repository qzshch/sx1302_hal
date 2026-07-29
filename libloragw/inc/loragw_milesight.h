/*
 * loragw_milesight.h — Milesight gateway board detection and hardware adaptation
 *
 * Transplanted from Milesight's fork of sx1302_hal (reverse-engineered from
 * lora_pkt_fwd binary). Provides:
 *   - Board detection via SX1302 GPIO registers (newpa/oldpa)
 *   - Enhanced SX1250 initialization (8-phase sequence with SetRegulatorMode)
 *   - TX power control via ur_pa direct SX1250 register writes
 *
 * Copyright: (C) 2026 — Adapted from Semtech sx1302_hal + Milesight modifications
 */

#ifndef _LORAGW_MILESIGHT_H
#define _LORAGW_MILESIGHT_H

#include <stdint.h>
#include <stdbool.h>

/* Board PA type detected at runtime via SX1302 GPIO pins */
typedef enum {
    MS_BOARD_UNKNOWN = 0,
    MS_BOARD_NEWPA = 1,  /* External PA circuit (hwver=0130/0150) */
    MS_BOARD_OLDPA = 2   /* Internal PA circuit (hwver=0200) */
} ms_board_pa_t;

/* Board detection result — populated by ms_detect_board() */
typedef struct {
    bool            detected;
    ms_board_pa_t   pa_type;
    int             duplex_mode;     /* from GPIO 0x117 bits[5:4]: 0=half, 1=full */
    uint8_t         gpio_in_l;      /* raw GPIO input low byte */
    uint8_t         gpio_in_h;      /* raw GPIO input high nibble */
} ms_board_info_t;

/**
 * Detect Milesight board type via SX1302 GPIO pins.
 * Must be called AFTER lgw_connect() but BEFORE sx1302_set_gpio(0x00).
 * Reads GPIO input pins to determine PA circuit type and duplex mode.
 */
int ms_detect_board(ms_board_info_t *info);

/**
 * Enhanced SX1250 initialization for Milesight boards.
 * Replaces upstream sx1250_setup() with an 8-phase sequence including:
 *   - SetRegulatorMode (LDO + DC-DC)
 *   - Double STANDBY_RC for power supply stabilization
 *   - Full calibration suite
 *   - SetDIOAsRfSwitch + SetPaDutyCycle + SetTxParams
 */
int ms_sx1250_setup(uint8_t rf_chain, uint32_t freq_hz, bool single_input_mode);

/**
 * Milesight TX power control — direct SX1250 register writes (ur_pa).
 * Called after upstream AGC_TX_PWR write in sx1302_send().
 * Writes SetPAConfig + SetTxParams + SetRFSwitchCtrl + SetPaDutyCycle
 * directly to the SX1250 radio.
 */
int ms_tx_pa_control(uint8_t rf_chain, uint8_t pa_gain, uint8_t pwr_idx);

/**
 * Get the board detection result (for use by other HAL functions).
 */
const ms_board_info_t* ms_get_board_info(void);

#endif /* _LORAGW_MILESIGHT_H */
