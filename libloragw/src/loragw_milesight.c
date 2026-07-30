/*
 * loragw_milesight.c — Milesight gateway board detection and hardware adaptation
 *
 * Board detection transplanted from native lora_pkt_fwd (0x413be0-0x413e38).
 * SX1250 init transplanted from native lora_pkt_fwd (0x421800-0x421b54).
 * TX ur_pa transplanted from native lora_pkt_fwd (0x417d70-0x417f3c).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "loragw_milesight.h"
#include "loragw_com.h"
#include "loragw_aux.h"
#include "loragw_sx1250.h"
#include "sx1250_defs.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* SX1302 GPIO register addresses (raw SPI, not register ID indices) */
#define SX1302_GPIO_DIR_L       0x0112  /* GPIO direction low byte */
#define SX1302_GPIO_DIR_H       0x0113  /* GPIO direction high nibble */
#define SX1302_GPIO_CFG         0x0118  /* GPIO config (register mode) */
#define SX1302_GPIO_OE          0x0119  /* GPIO output enable */
#define SX1302_GPIO_IN_L        0x0116  /* GPIO input low byte (read-only) */
#define SX1302_GPIO_IN_H        0x0117  /* GPIO input high nibble (read-only) */

/* SX1250 opcodes not in sx1250_defs.h */
/* SX1250_SET_DIO_AS_RF_SWITCH removed — using WRITE_REGISTER instead */

/* ur_pa default value — extracted from native HAL TX gain LUT */
#define MS_DEFAULT_UR_PA_BYTE   0x1E

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static ms_board_info_t board_info = { .detected = false };

/* -------------------------------------------------------------------------- */
/* --- PRIVATE HELPERS ------------------------------------------------------ */

/*
 * Raw SX1302 register write via SPI (bypasses register abstraction layer).
 * This matches the native Milesight HAL which uses direct SPI addresses.
 */
static int sx1302_raw_w(uint16_t addr, uint8_t data) {
    return lgw_com_w(LGW_SPI_MUX_TARGET_SX1302, addr, data);
}

/*
 * Raw SX1302 register read via SPI.
 */
static int sx1302_raw_r(uint16_t addr, uint8_t *data) {
    return lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, addr, data);
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS ----------------------------------------------------- */

const ms_board_info_t* ms_get_board_info(void) {
    return &board_info;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int ms_detect_board(ms_board_info_t *info) {
    int err = 0;
    uint8_t gpio_l_1, gpio_l_2, gpio_h;
    int pa_bit_1, pa_bit_2;

    printf("INFO: === Milesight Board Detection ===\n");

    /*
     * Read-only approach: after lgw_connect(), the SX1302 GPIO pins are in
     * their default state. We read the GPIO input registers directly without
     * modifying any GPIO configuration, to avoid disturbing the SPI MUX path
     * to the SX1250 radios.
     *
     * The native HAL writes to registers 0x112-0x123 before reading, but
     * those raw SPI writes have been found to interfere with SX1250 SPI
     * communication in the upstream HAL context.
     */

    /*
     * Step 1: Read GPIO input register (low byte) — double-read for debounce.
     * Raw SPI address 0x0116 = GPIO input low byte.
     */
    err = sx1302_raw_r(SX1302_GPIO_IN_L, &gpio_l_1);
    if (err != 0) {
        printf("ERROR: Failed to read GPIO_IN_L (1st read)\n");
        info->detected = false;
        return -1;
    }

    wait_ms(100);

    err = sx1302_raw_r(SX1302_GPIO_IN_L, &gpio_l_2);
    if (err != 0) {
        printf("ERROR: Failed to read GPIO_IN_L (2nd read)\n");
        info->detected = false;
        return -1;
    }

    printf("INFO: GPIO_IN_L: read1=0x%02X read2=0x%02X\n", gpio_l_1, gpio_l_2);

    /*
     * Step 3: Determine PA type from GPIO_IN_L bit 0.
     * Native HAL: pa_bit = val1 & 1; if (pa_bit & pa_bit2) → newpa; else → oldpa
     * Both reads must agree (debounce).
     */
    pa_bit_1 = gpio_l_1 & 1;
    pa_bit_2 = gpio_l_2 & 1;

    if (pa_bit_1 && pa_bit_2) {
        info->pa_type = MS_BOARD_NEWPA;
        printf("INFO: Milesight NEWPA detected (external PA circuit)\n");
    } else {
        info->pa_type = MS_BOARD_OLDPA;
        printf("INFO: Milesight OLDPA detected (internal PA circuit) — full_duplex will be forced off\n");
    }

    /*
     * Step 4: Read duplex mode from GPIO_IN_H (only meaningful for newpa).
     * Read-only: just read the register without modifying GPIO configuration.
     */
    err = sx1302_raw_r(SX1302_GPIO_IN_H, &gpio_h);
    if (err != 0) {
        printf("WARNING: Failed to read GPIO_IN_H for duplex mode\n");
        gpio_h = 0;
    }

    /*
     * Extract duplex mode from GPIO input high byte.
     * Native HAL: duplex_type = (gpio_117 >> 4) & 0x3
     * The register value layout may differ from raw SPI address.
     */
    info->duplex_mode = (gpio_h >> 1) & 0x3;  /* bits[2:1] of 4-bit register */
    printf("INFO: GPIO_IN_H=0x%02X, duplex_mode=%d\n", gpio_h, info->duplex_mode);

    /* Store results */
    info->gpio_in_l = gpio_l_1;
    info->gpio_in_h = gpio_h;
    info->detected = true;

    /* Copy to module-level state */
    memcpy(&board_info, info, sizeof(ms_board_info_t));

    printf("INFO: === Board Detection Complete: pa=%s duplex=%d ===\n",
           (info->pa_type == MS_BOARD_NEWPA) ? "NEWPA" : "OLDPA",
           info->duplex_mode);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int ms_sx1250_setup(uint8_t rf_chain, uint32_t freq_hz, bool single_input_mode) {
    uint8_t buff[16];
    int err = 0;
    int retry;

    printf("INFO: Milesight SX1250_%u setup (freq=%u Hz)\n", rf_chain, freq_hz);

    /*
     * Phase 1: STANDBY_RC with retry.
     * The upstream sx1250_setup() already has retry (from our v2 patch).
     * We keep the same approach but with the native HAL's timing.
     */
    for (retry = 0; retry < 5; retry++) {
        buff[0] = (uint8_t)STDBY_RC;
        err |= sx1250_reg_w(SET_STANDBY, buff, 1, rf_chain);
        wait_ms(50 + retry * 50);

        buff[0] = 0x00;
        err |= sx1250_reg_r(GET_STATUS, buff, 1, rf_chain);
        printf("INFO: MS SX1250_%u STANDBY_RC attempt %d: status=0x%02X (mode=%u)\n",
               rf_chain, retry + 1, buff[0], TAKE_N_BITS_FROM(buff[0], 4, 3));
        if (TAKE_N_BITS_FROM(buff[0], 4, 3) == 0x02) {
            break;
        }
    }
    if (retry >= 5) {
        printf("ERROR: MS SX1250_%u failed STANDBY_RC after %d attempts\n", rf_chain, retry);
        return -1;
    }

    /*
     * Phase 2: SetRegulatorMode — CRITICAL STEP missing from upstream.
     * Native HAL sends this between the two STANDBY_RC calls.
     * 0x7F = enable both LDO and DC-DC converter for all internal blocks.
     *
     * NOTE: The native binary uses opcode 0x89 here, but the SX1250 datasheet
     * defines SET_REGULATORMODE as 0x96. The native HAL might use a different
     * opcode mapping. We use the standard SET_REGULATORMODE (0x96).
     * If this doesn't work, try (sx1250_op_code_t)0x89 as fallback.
     */
    buff[0] = 0x01;  /* 0x01 = use DC-DC + LDO (RegDCDC+LDO) */
    err |= sx1250_reg_w(SET_REGULATORMODE, buff, 1, rf_chain);
    wait_ms(10);

    /*
     * Phase 3: Second STANDBY_RC → STANDBY_XOSC.
     * The native HAL does a second STANDBY_RC before transitioning to XOSC.
     * This ensures the power supply is stable before the crystal oscillator starts.
     */
    buff[0] = (uint8_t)STDBY_RC;
    err |= sx1250_reg_w(SET_STANDBY, buff, 1, rf_chain);
    wait_ms(10);

    for (retry = 0; retry < 5; retry++) {
        buff[0] = (uint8_t)STDBY_XOSC;
        err |= sx1250_reg_w(SET_STANDBY, buff, 1, rf_chain);
        wait_ms(50 + retry * 50);

        buff[0] = 0x00;
        err |= sx1250_reg_r(GET_STATUS, buff, 1, rf_chain);
        printf("INFO: MS SX1250_%u STANDBY_XOSC attempt %d: status=0x%02X (mode=%u)\n",
               rf_chain, retry + 1, buff[0], TAKE_N_BITS_FROM(buff[0], 4, 3));
        if (TAKE_N_BITS_FROM(buff[0], 4, 3) == 0x03) {
            break;
        }
    }
    if (retry >= 5) {
        printf("ERROR: MS SX1250_%u failed STANDBY_XOSC after %d attempts\n", rf_chain, retry);
        return -1;
    }

    /*
     * Phase 4: Full calibration (all blocks).
     */
    buff[0] = 0x7F;  /* Calibrate all: RC64K, RC13M, PLL, ADC, Img, BIAS, DAC */
    err |= sx1250_reg_w(CALIBRATE, buff, 1, rf_chain);
    wait_ms(10);

    /*
     * Phase 5: Image calibration (frequency-dependent).
     * Reuse upstream sx1250_calibrate() which handles the frequency→parameter mapping.
     */
    err |= sx1250_calibrate(rf_chain, freq_hz);

    /*
     * Phase 6: SetDIOAsRfSwitch — write SX1250 registers via WRITE_REGISTER.
     * Native HAL: helper_A(0x0D, {...}, ..., rf_chain)
     * 0x0D = WRITE_REGISTER, parameters are [addr_hi, addr_lo, value]
     */
    /* DIO5 enable for RF switch control */
    buff[0] = 0x05; buff[1] = 0x80; buff[2] = 0x08;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    /* DIO6 enable for RF switch control */
    buff[0] = 0x05; buff[1] = 0x81; buff[2] = 0x08;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    /* RF switch control: DIO5=RxEn, DIO6=TxEn */
    buff[0] = 0x05; buff[1] = 0x82; buff[2] = 0xF0;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    /* RfSwCtrl for radio 0 */
    buff[0] = 0x00; buff[1] = 0x59; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /*
     * Phase 7: SetTxParams — initial TX power.
     * Will be overridden per-packet by ms_tx_pa_control() via AGC.
     * Using 14 dBm as safe initial value (pa_gain=0, internal PA).
     */
    buff[0] = 14;    /* power in dBm */
    buff[1] = (uint8_t)SET_RAMP_200U;  /* ramp time */
    err |= sx1250_reg_w(SET_TX_PARAMS, buff, 2, rf_chain);

    /*
     * Phase 8: Radio configuration for RX — MUST match upstream order exactly.
     * Order: bitrate → DIO → gain → frequency → freq_offset → SET_RX → single_input → FPGA_MODE
     */

    /* Set max bitrate (to lower TX→FS switch time) */
    buff[0] = 0x06; buff[1] = 0xA1; buff[2] = 0x01;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    buff[0] = 0x06; buff[1] = 0xA2; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    buff[0] = 0x06; buff[1] = 0xA3; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /* Configure DIOs */
    buff[0] = 0x05; buff[1] = 0x82; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    buff[0] = 0x05; buff[1] = 0x83; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    buff[0] = 0x05; buff[1] = 0x84; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    buff[0] = 0x05; buff[1] = 0x85; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    buff[0] = 0x05; buff[1] = 0x80; buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /* Fixed gain */
    buff[0] = 0x08; buff[1] = 0xB6; buff[2] = 0x2A;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /* Set frequency */
    {
        int32_t freq_reg = SX1250_FREQ_TO_REG(freq_hz);
        buff[0] = (uint8_t)(freq_reg >> 24);
        buff[1] = (uint8_t)(freq_reg >> 16);
        buff[2] = (uint8_t)(freq_reg >> 8);
        buff[3] = (uint8_t)(freq_reg >> 0);
        err |= sx1250_reg_w(SET_RF_FREQUENCY, buff, 4, rf_chain);
    }

    /* Set frequency offset to 0 — CRITICAL: upstream always does this */
    buff[0] = 0x08; buff[1] = 0x8F; buff[2] = 0x00; buff[3] = 0x00; buff[4] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 5, rf_chain);

    /* Enter RX continuous mode — provides clock to SX1302 */
    buff[0] = 0xFF; buff[1] = 0xFF; buff[2] = 0xFF;
    err |= sx1250_reg_w(SET_RX, buff, 3, rf_chain);

    /* Single input mode (differential/SE) — from upstream */
    if (single_input_mode) {
        buff[0] = 0x08; buff[1] = 0xE2; buff[2] = 0x0D;
        err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    }

    /* FPGA_MODE_RX — from upstream */
    buff[0] = 0x05; buff[1] = 0x87; buff[2] = 0x0B;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    printf("INFO: MS SX1250_%u setup complete (err=%d)\n", rf_chain, err);
    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int ms_tx_pa_control(uint8_t rf_chain, uint8_t pa_gain, uint8_t pwr_idx) {
    uint8_t composite;
    uint8_t buff[4];
    int err = 0;

    /*
     * Compute ur_pa composite control byte.
     * Native HAL: composite = ur_pa_byte | (pa_byte << 6)
     *   - bits[7:6] = pa_gain (0-3)
     *   - bits[5:0] = ur_pa_byte (MS_DEFAULT_UR_PA_BYTE = 0x1E)
     */
    composite = MS_DEFAULT_UR_PA_BYTE | (pa_gain << 6);

    /*
     * Write SX1250 PA configuration registers directly.
     * Native HAL uses WRITE_REGISTER (opcode 0x0D) with register addresses:
     *   Radio 0: SetPAConfig=0x0046, SetTxParams=0x004C, RFSwitch=0x0059,
     *            PaDutyCycle=0x005C/5D/5E
     *   Radio 1: SetPAConfig=0x00B2, SetTxParams=0x00B8, RFSwitch=0x00C5,
     *            PaDutyCycle=0x00C8/C9/CA
     *
     * Register addresses are 16-bit, written as [addr_hi, addr_lo, value]
     * via WRITE_REGISTER opcode.
     */
    uint16_t reg_pa_config  = (rf_chain == 0) ? 0x0046 : 0x00B2;
    uint16_t reg_pa_config2 = (rf_chain == 0) ? 0x0056 : 0x00C2;
    uint16_t reg_tx_params  = (rf_chain == 0) ? 0x004C : 0x00B8;
    uint16_t reg_rf_switch  = (rf_chain == 0) ? 0x0059 : 0x00C5;
    uint16_t reg_duty_hi    = (rf_chain == 0) ? 0x005C : 0x00C8;
    uint16_t reg_duty_mid   = (rf_chain == 0) ? 0x005D : 0x00C9;
    uint16_t reg_duty_lo    = (rf_chain == 0) ? 0x005E : 0x00CA;

    /* SetPAConfig — radio-specific identifier */
    buff[0] = (uint8_t)(reg_pa_config >> 8);
    buff[1] = (uint8_t)(reg_pa_config & 0xFF);
    buff[2] = (rf_chain == 0) ? 0x00 : 0x01;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /* SetPAConfig second write */
    buff[0] = (uint8_t)(reg_pa_config2 >> 8);
    buff[1] = (uint8_t)(reg_pa_config2 & 0xFF);
    buff[2] = 0x01;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /* SetTxParams — composite PA control byte */
    buff[0] = (uint8_t)(reg_tx_params >> 8);
    buff[1] = (uint8_t)(reg_tx_params & 0xFF);
    buff[2] = composite;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /* SetRFSwitchCtrl */
    buff[0] = (uint8_t)(reg_rf_switch >> 8);
    buff[1] = (uint8_t)(reg_rf_switch & 0xFF);
    buff[2] = 0x00;
    err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

    /*
     * SetPaDutyCycle — 3-byte value derived from pwr_idx.
     * Native HAL uses a fixed-point multiply:
     *   magic = 0x431bde82d7b634db
     *   dc = (pwr_idx << 18) * magic >> (23 + 32)
     * This approximates: dc ≈ pwr_idx * 4
     */
    {
        uint32_t duty_cycle = (uint32_t)pwr_idx * 4;
        buff[0] = (uint8_t)(reg_duty_hi >> 8);
        buff[1] = (uint8_t)(reg_duty_hi & 0xFF);
        buff[2] = (uint8_t)((duty_cycle >> 16) & 0xFF);
        err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

        buff[0] = (uint8_t)(reg_duty_mid >> 8);
        buff[1] = (uint8_t)(reg_duty_mid & 0xFF);
        buff[2] = (uint8_t)((duty_cycle >> 8) & 0xFF);
        err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);

        buff[0] = (uint8_t)(reg_duty_lo >> 8);
        buff[1] = (uint8_t)(reg_duty_lo & 0xFF);
        buff[2] = (uint8_t)(duty_cycle & 0xFF);
        err |= sx1250_reg_w(WRITE_REGISTER, buff, 3, rf_chain);
    }

    printf("INFO: MS TX ur_pa: chain=%u pa=%u pwr_idx=%u composite=0x%02X duty=%u (err=%d)\n",
           rf_chain, pa_gain, pwr_idx, composite, pwr_idx * 4, err);

    return err;
}
