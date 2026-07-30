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
#define SX1302_GPIO_IN_H        0x0116  /* GPIO input high nibble (GPIO 15-12, read-only) */
#define SX1302_GPIO_IN_L        0x0117  /* GPIO input low byte (GPIO 7-0, read-only) */

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
    uint8_t val1, val2;
    int pa_bit_1, pa_bit_2;
    bool pa_detected = false;

    printf("INFO: === Milesight Board Detection (native HAL sequence) ===\n");

    /*
     * Phase 1: 14 GPIO configuration writes — MUST match native HAL exactly.
     * Native pkt_fw (0x413be0) writes these registers before any GPIO read.
     * Without proper config, GPIO input pins float → always read 0x00.
     *
     * Register map (raw SPI addresses):
     *   0x011A-0x011D = GPIO_OUT_H/L  (output value clear)
     *   0x011E-0x0121 = GPIO_PD_H/L   (pull-down disable)
     *   0x0112        = GPIO_DIR_L     (direction low byte → input)
     *   0x0113        = GPIO_DIR_H     (direction high nibble → input)
     *   0x0119        = GPIO_OE        (output enable)
     *   0x0122-0x0123 = GPIO_SEL       (pin selection clear)
     *   0x0118        = GPIO_CFG       (register mode = 0xC3)
     */

    /* GPIO_OUT_H[15:12] = 0 */
    err |= sx1302_raw_w(0x011A, 0x00);
    /* GPIO_OUT_H[11:8] = 0 */
    err |= sx1302_raw_w(0x011B, 0x00);
    /* GPIO_OUT_L[7:4] = 0 */
    err |= sx1302_raw_w(0x011C, 0x00);
    /* GPIO_OUT_L[3:0] = 0 */
    err |= sx1302_raw_w(0x011D, 0x00);
    /* GPIO_PD_H[15:12] = 0 (disable pull-down) */
    err |= sx1302_raw_w(0x011E, 0x00);
    /* GPIO_PD_H[11:8] = 0 */
    err |= sx1302_raw_w(0x011F, 0x00);
    /* GPIO_PD_L[7:4] = 0 */
    err |= sx1302_raw_w(0x0120, 0x00);
    /* GPIO_PD_L[3:0] = 0 */
    err |= sx1302_raw_w(0x0121, 0x00);
    /* GPIO_DIR_H[15:12] = 0 → set as input */
    err |= sx1302_raw_w(0x0113, 0x00);
    /* GPIO_OE = 0x40 */
    err |= sx1302_raw_w(0x0119, 0x40);
    /* GPIO_SEL_8_11 = 0 */
    err |= sx1302_raw_w(0x0122, 0x00);
    /* GPIO_SEL extension = 0 */
    err |= sx1302_raw_w(0x0123, 0x00);
    /* GPIO_DIR_L[7:0] = 0 → all input */
    err |= sx1302_raw_w(0x0112, 0x00);
    /* GPIO_CFG = 0xC3 (register mode) */
    err |= sx1302_raw_w(0x0118, 0xC3);

    if (err != 0) {
        printf("WARNING: GPIO config writes had errors (err=%d) — continuing\n", err);
    }

    wait_ms(10);  /* Let pin levels stabilize */

    /*
     * Phase 2: Two-stage PA detection (native HAL logic).
     *
     * Stage 1: Read GPIO_IN_H (0x0116) bit 0 — double-read debounce.
     *   If both reads have bit0=1 → NEWPA (fast path).
     *
     * Stage 2 (fallback): Read GPIO_IN_L (0x0117) bit 6 — double-read debounce.
     *   If both reads have bit6=1 → NEWPA.
     *   Otherwise → OLDPA.
     */

    /* Stage 1: Quick PA detect from GPIO_IN_H (0x0116) bit 0 */
    err = sx1302_raw_r(SX1302_GPIO_IN_H, &val1);
    if (err != 0) {
        printf("ERROR: Failed to read GPIO_IN_H (0x0116) stage 1\n");
        info->detected = false;
        return -1;
    }
    wait_ms(100);
    err = sx1302_raw_r(SX1302_GPIO_IN_H, &val2);
    if (err != 0) {
        printf("ERROR: Failed to read GPIO_IN_H (0x0116) stage 1 (2nd)\n");
        info->detected = false;
        return -1;
    }

    printf("INFO: PA stage1: GPIO_IN_H(0x0116) read1=0x%02X read2=0x%02X\n", val1, val2);

    pa_bit_1 = val1 & 0x1;  /* bit 0 */
    pa_bit_2 = val2 & 0x1;  /* bit 0 */

    if (pa_bit_1 && pa_bit_2) {
        info->pa_type = MS_BOARD_NEWPA;
        pa_detected = true;
        printf("INFO: NEWPA detected via GPIO_IN_H bit0\n");
    }

    /* Stage 2 (fallback): GPIO_IN_L (0x0117) bit 6 */
    if (!pa_detected) {
        err = sx1302_raw_r(SX1302_GPIO_IN_L, &val1);
        if (err != 0) {
            printf("ERROR: Failed to read GPIO_IN_L (0x0117) stage 2\n");
            info->detected = false;
            return -1;
        }
        wait_ms(100);
        err = sx1302_raw_r(SX1302_GPIO_IN_L, &val2);
        if (err != 0) {
            printf("ERROR: Failed to read GPIO_IN_L (0x0117) stage 2 (2nd)\n");
            info->detected = false;
            return -1;
        }

        printf("INFO: PA stage2: GPIO_IN_L(0x0117) read1=0x%02X read2=0x%02X\n", val1, val2);

        pa_bit_1 = (val1 >> 6) & 0x1;  /* bit 6 */
        pa_bit_2 = (val2 >> 6) & 0x1;  /* bit 6 */

        if (pa_bit_1 && pa_bit_2) {
            info->pa_type = MS_BOARD_NEWPA;
            pa_detected = true;
            printf("INFO: NEWPA detected via GPIO_IN_L bit6\n");
        } else {
            info->pa_type = MS_BOARD_OLDPA;
            printf("INFO: OLDPA detected (both stages negative)\n");
        }
    }

    /*
     * Phase 3: Duplex mode detection (only for newpa).
     * Native HAL reconfigures GPIO before reading duplex:
     *   GPIO_DIR_H (0x0113) = 0x0F → GPIO 12-15 as output
     *   GPIO_OE (0x0119) = 0x30    → output enable bits
     * Then reads GPIO_IN_L (0x0117) and extracts bits[5:4].
     *
     * Native instruction: ubfx val, val, #4, #2  →  (val >> 4) & 0x3
     */
    if (info->pa_type == MS_BOARD_NEWPA) {
        /* Reconfigure GPIO for duplex reading */
        sx1302_raw_w(0x0113, 0x0F);   /* GPIO_DIR_H = 0x0F */
        sx1302_raw_w(0x0119, 0x30);   /* GPIO_OE = 0x30 */
        wait_ms(100);

        err = sx1302_raw_r(SX1302_GPIO_IN_L, &val1);  /* Read 0x0117 */
        if (err != 0) {
            printf("WARNING: Failed to read duplex mode\n");
            info->duplex_mode = 0;
        } else {
            info->duplex_mode = (val1 >> 4) & 0x3;  /* bits[5:4] */
            printf("INFO: Duplex mode: GPIO_IN_L(0x0117)=0x%02X → duplex=%d\n",
                   val1, info->duplex_mode);
        }
    } else {
        info->duplex_mode = 0;  /* OLDPA is always half-duplex */
        printf("INFO: OLDPA → duplex_mode=0 (half-duplex)\n");
    }

    /* Store raw values for diagnostics */
    sx1302_raw_r(SX1302_GPIO_IN_H, &info->gpio_in_h);  /* 0x0116 */
    sx1302_raw_r(SX1302_GPIO_IN_L, &info->gpio_in_l);  /* 0x0117 */

    info->detected = true;

    /* Copy to module-level state */
    memcpy(&board_info, info, sizeof(ms_board_info_t));

    printf("INFO: === Board Detection Complete: pa=%s duplex=%d GPIO_H=0x%02X GPIO_L=0x%02X ===\n",
           (info->pa_type == MS_BOARD_NEWPA) ? "NEWPA" : "OLDPA",
           info->duplex_mode, info->gpio_in_h, info->gpio_in_l);

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
