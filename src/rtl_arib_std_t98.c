/*
 * rtl_arib_std_t98 - ARIB STD-T98 Digital Convenience Radio Demodulator
 * 
 * Demodulates Japan's Digital Convenience Radio (デジタル簡易無線) signals
 * on the 351 MHz band using π/4-QPSK modulation as specified in ARIB STD-T98.
 *
 * Copyright (C) 2026 tanakasan3
 * Based on rtl_fm.c from rtl-sdr-blog
 *
 * Original rtl_fm authors:
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

/*
 * ARIB STD-T98 Technical Specifications
 * 
 * Frequency Band:     351.03125 - 351.63125 MHz
 * Channel Spacing:    6.25 kHz
 * Modulation:         π/4-QPSK (π/4 Shifted QPSK)
 * Symbol Rate:        4,800 symbols/sec
 * Bit Rate:           9,600 bps (2 bits per symbol)
 * Occupied Bandwidth: ≤5.8 kHz
 * Roll-off Factor:    0.2
 * Frame Length:       40 ms (384 bits = 192 symbols)
 */

#define ARIB_SYMBOL_RATE        4800    /* symbols/sec */
#define ARIB_BIT_RATE           9600    /* bits/sec */
#define ARIB_FRAME_BITS         384     /* bits per 40ms frame */
#define ARIB_FRAME_SYMBOLS      192     /* symbols per frame */
#define ARIB_FRAME_MS           40      /* frame duration in ms */
#define ARIB_ROLLOFF            0.2     /* RRC roll-off factor */
#define ARIB_CHANNEL_SPACING    6250    /* Hz */

/* Sample rates - need sufficient oversampling for RRC filter */
#define SAMPLES_PER_SYMBOL      10
#define ARIB_SAMPLE_RATE        (ARIB_SYMBOL_RATE * SAMPLES_PER_SYMBOL)  /* 48000 Hz */
#define RTL_CAPTURE_RATE        240000  /* RTL-SDR capture rate (5x oversample) */

/* Sync word patterns */
#define SYNC_SC_BITS            20
#define SYNC_SC_PATTERN         0x1E56F     /* Service Channel sync: 20 bits */
#define SYNC_SB0_BITS           32
#define SYNC_SB0_PATTERN        0x2F94D06BUL /* Sync Burst sync: 32 bits */

/* Channel coding parameters */
#define CRC6_POLY               0x43        /* CRC-6: G(x) = 1 + x + x^6 */
#define CRC6_BITS               6
#define CONV_K                  6           /* Convolutional code constraint length */
#define CONV_RATE               2           /* 1/2 rate */
#define CONV_G1                 0x35        /* Generator polynomial 1: 1+D+D³+D⁵ */
#define CONV_G2                 0x3D        /* Generator polynomial 2: 1+D²+D³+D⁴+D⁵ */
#define CONV_TAIL_BITS          5           /* Tail bits for flushing encoder */

/* PN scrambler - PN(9,5) */
#define PN_LENGTH               9
#define PN_TAP1                 9           /* Tap position 1 (1-indexed) */
#define PN_TAP2                 5           /* Tap position 2 (1-indexed) */
#define PN_MASK                 0x1FF       /* 9-bit mask */

/* Channel frequencies for 351 MHz registered stations */
#define ARIB_BASE_FREQ          351200000   /* Channel 1 base frequency */
#define ARIB_CALLING_CHANNEL    15          /* 351.28750 MHz */

/* Buffer sizes */
#define DEFAULT_BUF_LENGTH      (16 * 16384)
#define RRC_FILTER_TAPS         65          /* RRC filter length */
#define MAX_FRAME_BUFFER        4096

/*
 * =============================================================================
 * Service Channel (SC) Frame Structure - 384 bits / 40ms
 * =============================================================================
 * | LP+R | Pa | TCH1 | RI(1/2) | SW | RI(2/2) | Undef | TCH2 |
 * |  30  | 2  |  96  |   56    | 20 |   14    |   6   | 160  |
 *
 * LP+R:    Linearizer preamble + burst guard (not decoded)
 * Pa:      Preamble "10"
 * TCH1:    Traffic Channel 1 (96 bits) - SCRAMBLED
 * RI(1/2): RICH part 1 (56 bits) - NOT scrambled
 * SW:      Sync Word (20 bits)
 * RI(2/2): RICH part 2 (14 bits) - NOT scrambled
 * Undef:   Undefined (6 bits) - SCRAMBLED
 * TCH2:    Traffic Channel 2 (160 bits) - SCRAMBLED
 */
#define SC_LP_R_START           0
#define SC_LP_R_END             30
#define SC_PREAMBLE_START       30
#define SC_PREAMBLE_END         32
#define SC_TCH1_START           32
#define SC_TCH1_END             128
#define SC_RICH1_START          128
#define SC_RICH1_END            184
#define SC_SW_START             184
#define SC_SW_END               204
#define SC_RICH2_START          204
#define SC_RICH2_END            218
#define SC_UNDEF_START          218
#define SC_UNDEF_END            224
#define SC_TCH2_START           224
#define SC_TCH2_END             384

#define SC_TCH1_BITS            (SC_TCH1_END - SC_TCH1_START)    /* 96 */
#define SC_RICH1_BITS           (SC_RICH1_END - SC_RICH1_START)  /* 56 */
#define SC_RICH2_BITS           (SC_RICH2_END - SC_RICH2_START)  /* 14 */
#define SC_RICH_TOTAL_BITS      (SC_RICH1_BITS + SC_RICH2_BITS)  /* 70 */
#define SC_TCH2_BITS            (SC_TCH2_END - SC_TCH2_START)    /* 160 */

/*
 * =============================================================================
 * Sync Burst (SB0) Frame Structure - 384 bits / 40ms
 * =============================================================================
 * | LP+R | Pb(1/2) | RI(1/2) | SW | RI(2/2) | Pb(2/2) | PICH | G |
 * |  40  |   88    |   56    | 32 |   14    |   26    | 120  | 8 |
 *
 * Sync Burst is NOT scrambled
 */
#define SB0_LP_R_START          0
#define SB0_LP_R_END            40
#define SB0_PB1_START           40
#define SB0_PB1_END             128
#define SB0_RICH1_START         128
#define SB0_RICH1_END           184
#define SB0_SW_START            184
#define SB0_SW_END              216
#define SB0_RICH2_START         216
#define SB0_RICH2_END           230
#define SB0_PB2_START           230
#define SB0_PB2_END             256
#define SB0_PICH_START          256
#define SB0_PICH_END            376
#define SB0_GUARD_START         376
#define SB0_GUARD_END           384

#define SB0_PICH_BITS           (SB0_PICH_END - SB0_PICH_START)  /* 120 */

/*
 * =============================================================================
 * RICH (Radio Information Channel) Structure
 * =============================================================================
 * After channel decoding (deinterleave -> Viterbi -> CRC), we get 24 info bits:
 * 
 * | F | M | S | UC | Undefined |
 * | 3 | 3 | 1 | 9  |     8     |
 *
 * F:  Frame type (3 bits)
 *     011 = Service Channel
 *     111 = Sync Burst
 *
 * M:  Communication mode (3 bits)
 *     001 = Voice
 *     010 = Data (no FEC)
 *     011 = Data (with FEC)
 *     100 = Sync Burst mode (for F=111)
 *     101 = End of transmission
 *
 * S:  Scramble code identifier (1 bit)
 *     0 = Scrambler initialized with User Code (UC)
 *     1 = Scrambler initialized with arbitrary value
 *
 * UC: User Code (9 bits)
 *     Used to initialize PN(9,5) scrambler when S=0
 */
#define RICH_F_BITS             3
#define RICH_M_BITS             3
#define RICH_S_BITS             1
#define RICH_UC_BITS            9
#define RICH_UNDEF_BITS         8
#define RICH_INFO_BITS          24

/* F field values */
#define RICH_F_SC               0b011   /* Service Channel */
#define RICH_F_SB0              0b111   /* Sync Burst */

/* M field values */
#define RICH_M_VOICE            0b001   /* Voice */
#define RICH_M_DATA1            0b010   /* Data without FEC */
#define RICH_M_DATA2            0b011   /* Data with FEC */
#define RICH_M_SB0              0b100   /* Sync Burst */
#define RICH_M_END              0b101   /* End of transmission */
#define RICH_M_DATA1_VOICE      0b110   /* TCH1=Data, TCH2=Voice */
#define RICH_M_DATA2_VOICE      0b111   /* TCH1=Data(FEC), TCH2=Voice */

/* S field values */
#define RICH_S_USER_CODE        0       /* Use UC for scrambler init */
#define RICH_S_ARBITRARY        1       /* Use arbitrary value */

/* Interleave parameters */
#define RICH_INTERLEAVE_ROWS    10      /* N=10 */
#define RICH_INTERLEAVE_COLS    7       /* 70 / 10 = 7 */
#define PICH_INTERLEAVE_ROWS    15      /* N=15 */
#define PICH_INTERLEAVE_COLS    8       /* 120 / 15 = 8 */

/*
 * =============================================================================
 * PICH (Parameter Information Channel) Structure
 * =============================================================================
 * After decoding, 48 info bits:
 *
 * | CSM (Call Sign) | Arbitrary |
 * |       36        |    12     |
 *
 * CSM: BCD-encoded 9-digit call sign (呼出名称)
 */
#define PICH_CSM_BITS           36
#define PICH_ARB_BITS           12
#define PICH_INFO_BITS          48

static volatile int do_exit = 0;

/* Root Raised Cosine filter coefficients */
static double rrc_coeffs[RRC_FILTER_TAPS];
static double rrc_delay_i[RRC_FILTER_TAPS];
static double rrc_delay_q[RRC_FILTER_TAPS];

/*
 * Decoded RICH information
 */
struct rich_info {
    uint8_t f_field;            /* Frame type (3 bits) */
    uint8_t m_field;            /* Communication mode (3 bits) */
    uint8_t s_field;            /* Scramble identifier (1 bit) */
    uint16_t user_code;         /* User Code (9 bits) */
    int crc_valid;              /* CRC check passed */
};

/*
 * Decoded PICH information
 */
struct pich_info {
    char call_sign[10];         /* 9-digit call sign + null */
    uint64_t call_sign_raw;     /* Raw 36-bit BCD value */
    uint16_t arbitrary;         /* Arbitrary bits (12 bits) */
    int crc_valid;              /* CRC check passed */
};

/*
 * Decoded frame information
 */
struct frame_info {
    int frame_type;             /* 1=SC, 2=SB0 */
    struct rich_info rich;
    struct pich_info pich;      /* Only valid for SB0 */
    uint8_t tch1[SC_TCH1_BITS]; /* TCH1 bits (descrambled) */
    uint8_t tch2[SC_TCH2_BITS]; /* TCH2 bits (descrambled) */
    int tch1_valid;
    int tch2_valid;
};

/*
 * PN(9,5) Scrambler/Descrambler state
 */
struct pn_scrambler {
    uint16_t shift_reg;         /* 9-bit shift register */
    uint16_t init_value;        /* Initial value (UC or arbitrary) */
};

/*
 * Viterbi decoder state
 */
struct viterbi_state {
    uint32_t path_metric[32];   /* 2^(K-1) = 32 states */
    uint32_t path[32];
    int len;
};

/*
 * Main receiver state
 */
struct arib_state {
    /* RTL-SDR device */
    rtlsdr_dev_t *dev;
    int dev_index;
    uint32_t freq;
    int gain;
    int ppm_error;
    int enable_biastee;
    
    /* Channel selection */
    int channel;
    
    /* Sample buffers */
    int16_t *iq_buffer;
    int iq_len;
    
    /* Demodulation state */
    double prev_phase;
    int symbol_count;
    int samples_since_symbol;
    
    /* Symbol timing recovery */
    double timing_error;
    double timing_nco;
    double timing_gain;
    
    /* Carrier recovery */
    double carrier_phase;
    double carrier_freq_offset;
    double carrier_gain;
    
    /* Bit buffer for sync detection */
    uint64_t bit_buffer;
    int bit_count;
    
    /* Frame buffer */
    uint8_t frame_bits[ARIB_FRAME_BITS];
    int frame_bit_idx;
    int frame_synced;
    int frame_type;             /* 1=SC, 2=SB0 */
    
    /* Current decoded frame */
    struct frame_info current_frame;
    
    /* PN descrambler */
    struct pn_scrambler descrambler;
    uint16_t current_user_code; /* Most recent UC from RICH */
    int user_code_valid;        /* Have we received a valid UC? */
    
    /* Statistics */
    uint64_t frames_received;
    uint64_t frames_decoded;
    uint64_t rich_crc_ok;
    uint64_t rich_crc_fail;
    uint64_t pich_crc_ok;
    uint64_t pich_crc_fail;
    
    /* Output */
    FILE *output_file;
    int raw_output;
    int verbose;
    int show_rich;              /* Show RICH decode info */
    
    /* Threading */
    pthread_t demod_thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int data_ready;
};

static struct arib_state state;

void usage(void)
{
    fprintf(stderr,
        "rtl_arib_std_t98 - ARIB STD-T98 Digital Convenience Radio Demodulator\n\n"
        "Demodulates Japan's 351 MHz Digital Convenience Radio (デジタル簡易無線)\n"
        "signals using π/4-QPSK modulation per ARIB STD-T98 specification.\n\n"
        "Usage:\trtl_arib_std_t98 [-options] [output_file]\n"
        "\t-c channel     Channel number 1-82 (default: 15 calling channel)\n"
        "\t               Channels 1-70:  351.20000 - 351.63125 MHz\n"
        "\t               Channels 71-82: 351.03125 - 351.10000 MHz\n"
        "\t-f frequency   Direct frequency in Hz (overrides -c)\n"
        "\t[-d device]    RTL-SDR device index (default: 0)\n"
        "\t[-g gain]      Tuner gain in dB (default: auto)\n"
        "\t[-p ppm]       PPM error correction (default: 0)\n"
        "\t[-T]           Enable bias-T\n"
        "\t[-r]           Raw bit output (no decoding)\n"
        "\t[-R]           Show RICH decode details\n"
        "\t[-v]           Verbose output\n"
        "\t[-h]           Show this help\n\n"
        "Output: Decoded frames written to stdout or specified file\n\n"
        "Examples:\n"
        "\trtl_arib_std_t98 -c 15              # Monitor calling channel\n"
        "\trtl_arib_std_t98 -c 15 -R           # Show RICH decoding\n"
        "\trtl_arib_std_t98 -f 351287500       # Direct frequency\n\n"
        "Frame Decoding:\n"
        "\t- Extracts RICH to get User Code (UC)\n"
        "\t- Uses UC to initialize PN(9,5) descrambler\n"
        "\t- Descrambles TCH1/TCH2 traffic channels\n"
        "\t- Decodes PICH for call sign (SB0 frames)\n\n"
        "Technical Specs (ARIB STD-T98):\n"
        "\tModulation:    π/4-QPSK\n"
        "\tSymbol Rate:   4,800 sym/sec\n"
        "\tBit Rate:      9,600 bps\n"
        "\tFrame Length:  40 ms (384 bits)\n"
        "\tRoll-off:      α=0.2\n"
        "\tScrambler:     PN(9,5) initialized with UC\n\n"
        "⚠️  Legal Notice: In Japan, operating on these frequencies requires\n"
        "   proper licensing. Comply with local radio regulations.\n\n");
    exit(1);
}

#ifdef _WIN32
BOOL WINAPI sighandler(int signum)
{
    if (CTRL_C_EVENT == signum) {
        fprintf(stderr, "Signal caught, exiting!\n");
        do_exit = 1;
        rtlsdr_cancel_async(state.dev);
        return TRUE;
    }
    return FALSE;
}
#else
static void sighandler(int signum)
{
    (void)signum;
    signal(SIGPIPE, SIG_IGN);
    fprintf(stderr, "Signal caught, exiting!\n");
    do_exit = 1;
    rtlsdr_cancel_async(state.dev);
}
#endif

/* =============================================================================
 * PN(9,5) Scrambler/Descrambler
 * =============================================================================
 * 
 * LFSR with feedback polynomial: x^9 + x^5 + 1
 * Feedback: output = bit[8] XOR bit[4] (0-indexed)
 * 
 * The scrambler is initialized with the User Code (UC) from RICH when S=0,
 * or with an arbitrary value when S=1.
 */

static void pn_init(struct pn_scrambler *pn, uint16_t init_value)
{
    pn->init_value = init_value & PN_MASK;
    pn->shift_reg = pn->init_value;
}

static void pn_reset(struct pn_scrambler *pn)
{
    pn->shift_reg = pn->init_value;
}

static uint8_t pn_clock(struct pn_scrambler *pn)
{
    /* Feedback: XOR of tap positions 9 and 5 (1-indexed) = bits 8 and 4 (0-indexed) */
    uint8_t feedback = ((pn->shift_reg >> (PN_TAP1 - 1)) ^ 
                        (pn->shift_reg >> (PN_TAP2 - 1))) & 1;
    
    /* Shift register left and insert feedback */
    pn->shift_reg = ((pn->shift_reg << 1) | feedback) & PN_MASK;
    
    return feedback;
}

static uint8_t pn_descramble_bit(struct pn_scrambler *pn, uint8_t bit)
{
    uint8_t pn_bit = pn_clock(pn);
    return bit ^ pn_bit;
}

static void pn_descramble_bits(struct pn_scrambler *pn, uint8_t *bits, int len)
{
    for (int i = 0; i < len; i++) {
        bits[i] = pn_descramble_bit(pn, bits[i]);
    }
}

/* =============================================================================
 * CRC-6 Encoder/Checker
 * =============================================================================
 * G(x) = 1 + x + x^6 = 0x43
 */

static void crc6_init(uint8_t *reg)
{
    memset(reg, 0, 6);
}

static void crc6_process_bit(uint8_t *reg, uint8_t bit)
{
    uint8_t feedback = bit ^ reg[5];
    
    /* Shift register */
    reg[5] = reg[4];
    reg[4] = reg[3];
    reg[3] = reg[2];
    reg[2] = reg[1];
    reg[1] = reg[0] ^ feedback;  /* Tap at position 1 */
    reg[0] = feedback;           /* Tap at position 0 */
}

static void crc6_compute(uint8_t *data, int num_bits, uint8_t *crc_out)
{
    uint8_t reg[6] = {0};
    
    for (int i = 0; i < num_bits; i++) {
        crc6_process_bit(reg, data[i]);
    }
    
    memcpy(crc_out, reg, 6);
}

static int crc6_check(uint8_t *data_with_crc, int total_bits)
{
    if (total_bits <= CRC6_BITS) return 0;
    
    int data_bits = total_bits - CRC6_BITS;
    uint8_t computed[6];
    crc6_compute(data_with_crc, data_bits, computed);
    
    /* Compare with received CRC */
    for (int i = 0; i < CRC6_BITS; i++) {
        if (computed[i] != data_with_crc[data_bits + i]) {
            return 0;
        }
    }
    return 1;
}

/* =============================================================================
 * Viterbi Decoder (R=1/2, K=6)
 * =============================================================================
 */

static int parity(int x)
{
    int p = 0;
    while (x) { p ^= (x & 1); x >>= 1; }
    return p;
}

static void viterbi_init(struct viterbi_state *v)
{
    memset(v, 0, sizeof(*v));
    for (int i = 1; i < 32; i++) {
        v->path_metric[i] = 0xFFFFFFFF;
    }
}

static void viterbi_decode_pair(struct viterbi_state *v, uint8_t sym0, uint8_t sym1)
{
    uint32_t new_metric[32];
    uint32_t new_path[32];
    
    for (int i = 0; i < 32; i++) {
        new_metric[i] = 0xFFFFFFFF;
        new_path[i] = 0;
    }
    
    for (int state = 0; state < 32; state++) {
        if (v->path_metric[state] == 0xFFFFFFFF) continue;
        
        for (int input = 0; input < 2; input++) {
            int next_state = ((state << 1) | input) & 0x1F;
            
            /* Calculate expected output */
            int encoder_state = (state << 1) | input;
            uint8_t exp0 = parity(encoder_state & CONV_G1);
            uint8_t exp1 = parity(encoder_state & CONV_G2);
            
            /* Branch metric (Hamming distance) */
            int bm = (sym0 != exp0) + (sym1 != exp1);
            
            uint32_t pm = v->path_metric[state] + bm;
            if (pm < new_metric[next_state]) {
                new_metric[next_state] = pm;
                new_path[next_state] = (v->path[state] << 1) | input;
            }
        }
    }
    
    memcpy(v->path_metric, new_metric, sizeof(new_metric));
    memcpy(v->path, new_path, sizeof(new_path));
    v->len++;
}

static int viterbi_decode(uint8_t *received, int num_symbols, uint8_t *decoded, int max_bits)
{
    struct viterbi_state v;
    viterbi_init(&v);
    
    for (int i = 0; i < num_symbols; i++) {
        viterbi_decode_pair(&v, received[2*i], received[2*i + 1]);
    }
    
    /* Find best ending state */
    int best_state = 0;
    uint32_t best_metric = v.path_metric[0];
    for (int i = 1; i < 32; i++) {
        if (v.path_metric[i] < best_metric) {
            best_metric = v.path_metric[i];
            best_state = i;
        }
    }
    
    /* Traceback */
    int num_bits = (v.len < max_bits) ? v.len : max_bits;
    for (int i = 0; i < num_bits; i++) {
        decoded[num_bits - 1 - i] = (v.path[best_state] >> i) & 1;
    }
    
    /* Remove tail bits */
    if (num_bits > CONV_TAIL_BITS) {
        num_bits -= CONV_TAIL_BITS;
    }
    
    return num_bits;
}

/* =============================================================================
 * Block Deinterleaver
 * =============================================================================
 * Write by columns, read by rows (reverse of transmitter)
 */

static void deinterleave(uint8_t *input, uint8_t *output, int rows, int cols)
{
    for (int c = 0; c < cols; c++) {
        for (int r = 0; r < rows; r++) {
            output[c * rows + r] = input[r * cols + c];
        }
    }
}

/* =============================================================================
 * RICH Decoder
 * =============================================================================
 * Chain: Received (70 bits) -> Deinterleave -> Viterbi -> CRC check
 */

static int decode_rich(uint8_t *rich_bits, struct rich_info *info)
{
    uint8_t deinterleaved[SC_RICH_TOTAL_BITS];
    uint8_t decoded[64];
    
    /* Deinterleave: 10 rows x 7 cols -> 7 rows x 10 cols */
    deinterleave(rich_bits, deinterleaved, RICH_INTERLEAVE_ROWS, RICH_INTERLEAVE_COLS);
    
    /* Viterbi decode: 70 bits -> 35 symbols -> ~30 info bits (with CRC+tail) */
    int num_symbols = SC_RICH_TOTAL_BITS / 2;
    int decoded_bits = viterbi_decode(deinterleaved, num_symbols, decoded, 64);
    
    /* Check CRC (24 info bits + 6 CRC bits = 30 bits) */
    if (decoded_bits >= 30) {
        info->crc_valid = crc6_check(decoded, 30);
    } else {
        info->crc_valid = 0;
    }
    
    /* Extract fields regardless of CRC (for debugging) */
    if (decoded_bits >= RICH_INFO_BITS) {
        /* F field: bits 0-2 */
        info->f_field = (decoded[0] << 2) | (decoded[1] << 1) | decoded[2];
        
        /* M field: bits 3-5 */
        info->m_field = (decoded[3] << 2) | (decoded[4] << 1) | decoded[5];
        
        /* S field: bit 6 */
        info->s_field = decoded[6];
        
        /* UC field: bits 7-15 (9 bits) */
        info->user_code = 0;
        for (int i = 0; i < RICH_UC_BITS; i++) {
            info->user_code = (info->user_code << 1) | decoded[7 + i];
        }
    }
    
    return info->crc_valid;
}

/* =============================================================================
 * PICH Decoder  
 * =============================================================================
 * Chain: Received (120 bits) -> Deinterleave -> Viterbi -> CRC check
 */

static int decode_pich(uint8_t *pich_bits, struct pich_info *info)
{
    uint8_t deinterleaved[SB0_PICH_BITS];
    uint8_t decoded[96];
    
    /* Deinterleave: 15 rows x 8 cols */
    deinterleave(pich_bits, deinterleaved, PICH_INTERLEAVE_ROWS, PICH_INTERLEAVE_COLS);
    
    /* Viterbi decode: 120 bits -> 60 symbols -> ~55 info bits */
    int num_symbols = SB0_PICH_BITS / 2;
    int decoded_bits = viterbi_decode(deinterleaved, num_symbols, decoded, 96);
    
    /* Check CRC (48 info bits + 1 null + 6 CRC = 55 bits) */
    /* Note: PICH adds a null bit before CRC */
    if (decoded_bits >= 55) {
        info->crc_valid = crc6_check(decoded, 55);
    } else {
        info->crc_valid = 0;
    }
    
    /* Extract fields */
    if (decoded_bits >= PICH_INFO_BITS) {
        /* CSM: bits 0-35 (36 bits, BCD encoded) */
        info->call_sign_raw = 0;
        for (int i = 0; i < PICH_CSM_BITS; i++) {
            info->call_sign_raw = (info->call_sign_raw << 1) | decoded[i];
        }
        
        /* Convert BCD to string */
        for (int d = 0; d < 9; d++) {
            int shift = (8 - d) * 4;
            int digit = (info->call_sign_raw >> shift) & 0xF;
            info->call_sign[d] = (digit <= 9) ? ('0' + digit) : '?';
        }
        info->call_sign[9] = '\0';
        
        /* Arbitrary bits: 36-47 */
        info->arbitrary = 0;
        for (int i = 0; i < PICH_ARB_BITS; i++) {
            info->arbitrary = (info->arbitrary << 1) | decoded[PICH_CSM_BITS + i];
        }
    }
    
    return info->crc_valid;
}

/* =============================================================================
 * Service Channel Frame Decoder
 * =============================================================================
 */

static void decode_service_channel(struct arib_state *s, uint8_t *frame)
{
    struct frame_info *fi = &s->current_frame;
    memset(fi, 0, sizeof(*fi));
    fi->frame_type = 1;
    
    /* Extract RICH (combine both parts) */
    uint8_t rich_bits[SC_RICH_TOTAL_BITS];
    memcpy(rich_bits, &frame[SC_RICH1_START], SC_RICH1_BITS);
    memcpy(rich_bits + SC_RICH1_BITS, &frame[SC_RICH2_START], SC_RICH2_BITS);
    
    /* Decode RICH */
    if (decode_rich(rich_bits, &fi->rich)) {
        s->rich_crc_ok++;
        
        /* Update current user code if S=0 (use UC for scrambling) */
        if (fi->rich.s_field == RICH_S_USER_CODE) {
            s->current_user_code = fi->rich.user_code;
            s->user_code_valid = 1;
        }
        
        if (s->show_rich || s->verbose) {
            fprintf(stderr, "RICH: F=%d M=%d S=%d UC=0x%03X (%d)\n",
                    fi->rich.f_field, fi->rich.m_field, fi->rich.s_field,
                    fi->rich.user_code, fi->rich.user_code);
        }
    } else {
        s->rich_crc_fail++;
        if (s->verbose) {
            fprintf(stderr, "RICH: CRC FAIL (F=%d M=%d S=%d UC=0x%03X)\n",
                    fi->rich.f_field, fi->rich.m_field, fi->rich.s_field,
                    fi->rich.user_code);
        }
    }
    
    /* Descramble TCH1 and TCH2 using UC */
    if (s->user_code_valid && fi->rich.s_field == RICH_S_USER_CODE) {
        /* Initialize descrambler with UC */
        pn_init(&s->descrambler, s->current_user_code);
        
        /* Copy and descramble TCH1 */
        memcpy(fi->tch1, &frame[SC_TCH1_START], SC_TCH1_BITS);
        pn_descramble_bits(&s->descrambler, fi->tch1, SC_TCH1_BITS);
        fi->tch1_valid = 1;
        
        /* Reset descrambler for second scrambled region */
        pn_reset(&s->descrambler);
        
        /* Descramble undefined + TCH2 */
        /* Note: Undefined bits (6) and TCH2 (160) are scrambled together */
        uint8_t tch2_region[SC_TCH2_END - SC_UNDEF_START];
        memcpy(tch2_region, &frame[SC_UNDEF_START], sizeof(tch2_region));
        pn_descramble_bits(&s->descrambler, tch2_region, sizeof(tch2_region));
        
        /* Extract TCH2 (after undefined bits) */
        memcpy(fi->tch2, tch2_region + (SC_TCH2_START - SC_UNDEF_START), SC_TCH2_BITS);
        fi->tch2_valid = 1;
        
        if (s->verbose) {
            fprintf(stderr, "  Descrambled with UC=0x%03X\n", s->current_user_code);
        }
    } else if (s->user_code_valid && fi->rich.s_field == RICH_S_ARBITRARY) {
        /* S=1: arbitrary scrambling - we can't descramble without knowing the value */
        memcpy(fi->tch1, &frame[SC_TCH1_START], SC_TCH1_BITS);
        memcpy(fi->tch2, &frame[SC_TCH2_START], SC_TCH2_BITS);
        fi->tch1_valid = 0;  /* Mark as not properly descrambled */
        fi->tch2_valid = 0;
        
        if (s->verbose) {
            fprintf(stderr, "  S=1: Arbitrary scrambling (cannot descramble)\n");
        }
    }
    
    s->frames_decoded++;
}

/* =============================================================================
 * Sync Burst Frame Decoder
 * =============================================================================
 */

static void decode_sync_burst(struct arib_state *s, uint8_t *frame)
{
    struct frame_info *fi = &s->current_frame;
    memset(fi, 0, sizeof(*fi));
    fi->frame_type = 2;
    
    /* Extract RICH */
    uint8_t rich_bits[SC_RICH_TOTAL_BITS];
    memcpy(rich_bits, &frame[SB0_RICH1_START], SC_RICH1_BITS);
    memcpy(rich_bits + SC_RICH1_BITS, &frame[SB0_RICH2_START], SC_RICH2_BITS);
    
    /* Decode RICH */
    if (decode_rich(rich_bits, &fi->rich)) {
        s->rich_crc_ok++;
        
        /* Update UC from Sync Burst */
        if (fi->rich.s_field == RICH_S_USER_CODE) {
            s->current_user_code = fi->rich.user_code;
            s->user_code_valid = 1;
        }
        
        if (s->show_rich || s->verbose) {
            fprintf(stderr, "RICH (SB0): F=%d M=%d S=%d UC=0x%03X\n",
                    fi->rich.f_field, fi->rich.m_field, fi->rich.s_field,
                    fi->rich.user_code);
        }
    } else {
        s->rich_crc_fail++;
    }
    
    /* Extract and decode PICH (Sync Burst only, not scrambled) */
    uint8_t pich_bits[SB0_PICH_BITS];
    memcpy(pich_bits, &frame[SB0_PICH_START], SB0_PICH_BITS);
    
    if (decode_pich(pich_bits, &fi->pich)) {
        s->pich_crc_ok++;
        
        if (s->show_rich || s->verbose) {
            fprintf(stderr, "PICH: Call Sign=%s\n", fi->pich.call_sign);
        }
    } else {
        s->pich_crc_fail++;
    }
    
    s->frames_decoded++;
}

/* =============================================================================
 * Root Raised Cosine Filter
 * =============================================================================
 */

static void generate_rrc_filter(double *coeffs, int num_taps, double alpha, int samples_per_symbol)
{
    int center = num_taps / 2;
    double sum = 0.0;
    double T = 1.0 / (double)samples_per_symbol;
    
    for (int i = 0; i < num_taps; i++) {
        double t = (double)(i - center) * T;
        double h;
        
        if (fabs(t) < 1e-10) {
            h = (1.0 - alpha + 4.0 * alpha / M_PI);
        } else if (fabs(fabs(t) - T / (4.0 * alpha)) < 1e-10) {
            h = (alpha / sqrt(2.0)) * (
                (1.0 + 2.0/M_PI) * sin(M_PI / (4.0 * alpha)) +
                (1.0 - 2.0/M_PI) * cos(M_PI / (4.0 * alpha))
            );
        } else {
            double sinc_term = sin(M_PI * t * (1.0 - alpha)) + 
                               4.0 * alpha * t * cos(M_PI * t * (1.0 + alpha));
            double denom = M_PI * t * (1.0 - pow(4.0 * alpha * t, 2));
            h = sinc_term / denom;
        }
        
        coeffs[i] = h;
        sum += h;
    }
    
    for (int i = 0; i < num_taps; i++) {
        coeffs[i] /= sum;
    }
}

static void rrc_filter(double *in_i, double *in_q, double *out_i, double *out_q)
{
    memmove(&rrc_delay_i[1], &rrc_delay_i[0], (RRC_FILTER_TAPS - 1) * sizeof(double));
    memmove(&rrc_delay_q[1], &rrc_delay_q[0], (RRC_FILTER_TAPS - 1) * sizeof(double));
    
    rrc_delay_i[0] = *in_i;
    rrc_delay_q[0] = *in_q;
    
    *out_i = 0.0;
    *out_q = 0.0;
    for (int i = 0; i < RRC_FILTER_TAPS; i++) {
        *out_i += rrc_delay_i[i] * rrc_coeffs[i];
        *out_q += rrc_delay_q[i] * rrc_coeffs[i];
    }
}

/* =============================================================================
 * Channel Frequency Calculation
 * =============================================================================
 */

static uint32_t channel_to_freq(int channel)
{
    if (channel >= 1 && channel <= 70) {
        return 351200000 + (channel - 1) * ARIB_CHANNEL_SPACING;
    } else if (channel >= 71 && channel <= 82) {
        return 351031250 + (channel - 71) * ARIB_CHANNEL_SPACING;
    }
    return 351287500;
}

/* =============================================================================
 * π/4-QPSK Demodulation
 * =============================================================================
 */

static int pi4_qpsk_demod(double i_sample, double q_sample, double *prev_phase)
{
    double phase = atan2(q_sample, i_sample);
    double delta_phase = phase - *prev_phase;
    
    while (delta_phase > M_PI) delta_phase -= 2.0 * M_PI;
    while (delta_phase < -M_PI) delta_phase += 2.0 * M_PI;
    
    *prev_phase = phase;
    
    /* Map phase change to dibit:
     * Δφ ≈ +π/4  -> 00
     * Δφ ≈ +3π/4 -> 01  
     * Δφ ≈ -3π/4 -> 11
     * Δφ ≈ -π/4  -> 10
     */
    int dibit;
    if (delta_phase >= 0 && delta_phase < M_PI/2) {
        dibit = 0b00;
    } else if (delta_phase >= M_PI/2 && delta_phase <= M_PI) {
        dibit = 0b01;
    } else if (delta_phase < 0 && delta_phase >= -M_PI/2) {
        dibit = 0b10;
    } else {
        dibit = 0b11;
    }
    
    return dibit;
}

/* =============================================================================
 * Sync Detection
 * =============================================================================
 */

static int check_sync(struct arib_state *s)
{
    uint32_t sc_mask = (1UL << SYNC_SC_BITS) - 1;
    if ((s->bit_buffer & sc_mask) == SYNC_SC_PATTERN) {
        return 1;
    }
    
    uint64_t sb0_mask = (1ULL << SYNC_SB0_BITS) - 1;
    if ((s->bit_buffer & sb0_mask) == SYNC_SB0_PATTERN) {
        return 2;
    }
    
    return 0;
}

/* =============================================================================
 * Frame Decoder Dispatch
 * =============================================================================
 */

static void decode_frame(struct arib_state *s)
{
    s->frames_received++;
    
    if (s->verbose) {
        fprintf(stderr, "\n=== Frame %lu: %s ===\n",
                (unsigned long)s->frames_received,
                s->frame_type == 1 ? "Service Channel" : "Sync Burst");
    }
    
    /* Raw output mode */
    if (s->raw_output && s->output_file) {
        uint8_t byte = 0;
        int bit_in_byte = 0;
        
        for (int i = 0; i < s->frame_bit_idx; i++) {
            byte = (byte << 1) | (s->frame_bits[i] & 1);
            bit_in_byte++;
            if (bit_in_byte == 8) {
                fputc(byte, s->output_file);
                byte = 0;
                bit_in_byte = 0;
            }
        }
        if (bit_in_byte > 0) {
            byte <<= (8 - bit_in_byte);
            fputc(byte, s->output_file);
        }
        fflush(s->output_file);
        return;
    }
    
    /* Decode based on frame type */
    if (s->frame_type == 1) {
        decode_service_channel(s, s->frame_bits);
    } else {
        decode_sync_burst(s, s->frame_bits);
    }
    
    /* Output descrambled TCH if valid */
    if (!s->raw_output && s->output_file) {
        if (s->current_frame.tch1_valid) {
            for (int i = 0; i < SC_TCH1_BITS; i += 8) {
                uint8_t byte = 0;
                for (int b = 0; b < 8 && (i + b) < SC_TCH1_BITS; b++) {
                    byte = (byte << 1) | s->current_frame.tch1[i + b];
                }
                fputc(byte, s->output_file);
            }
        }
        if (s->current_frame.tch2_valid) {
            for (int i = 0; i < SC_TCH2_BITS; i += 8) {
                uint8_t byte = 0;
                for (int b = 0; b < 8 && (i + b) < SC_TCH2_BITS; b++) {
                    byte = (byte << 1) | s->current_frame.tch2[i + b];
                }
                fputc(byte, s->output_file);
            }
        }
        fflush(s->output_file);
    }
}

/* =============================================================================
 * Symbol Processing
 * =============================================================================
 */

static void process_symbols(struct arib_state *s, int16_t *samples, int len)
{
    for (int i = 0; i < len - 1; i += 2) {
        double i_raw = (double)samples[i] / 32768.0;
        double q_raw = (double)samples[i + 1] / 32768.0;
        
        double i_filt, q_filt;
        rrc_filter(&i_raw, &q_raw, &i_filt, &q_filt);
        
        s->samples_since_symbol++;
        
        if (s->samples_since_symbol >= SAMPLES_PER_SYMBOL) {
            s->samples_since_symbol = 0;
            
            int dibit = pi4_qpsk_demod(i_filt, q_filt, &s->prev_phase);
            
            uint8_t bit1 = (dibit >> 1) & 1;
            uint8_t bit0 = dibit & 1;
            
            s->bit_buffer = (s->bit_buffer << 2) | dibit;
            s->bit_count += 2;
            
            if (!s->frame_synced) {
                int sync_type = check_sync(s);
                if (sync_type) {
                    s->frame_synced = 1;
                    s->frame_type = sync_type;
                    s->frame_bit_idx = 0;
                    
                    if (s->verbose) {
                        fprintf(stderr, "SYNC: %s detected\n",
                                sync_type == 1 ? "SC" : "SB0");
                    }
                }
            } else {
                if (s->frame_bit_idx < ARIB_FRAME_BITS) {
                    s->frame_bits[s->frame_bit_idx++] = bit1;
                }
                if (s->frame_bit_idx < ARIB_FRAME_BITS) {
                    s->frame_bits[s->frame_bit_idx++] = bit0;
                }
                
                if (s->frame_bit_idx >= ARIB_FRAME_BITS) {
                    decode_frame(s);
                    s->frame_synced = 0;
                    s->frame_bit_idx = 0;
                }
            }
            
            s->symbol_count++;
        }
    }
}

/* =============================================================================
 * RTL-SDR Callback
 * =============================================================================
 */

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    struct arib_state *s = ctx;
    
    if (do_exit) return;
    if (!ctx) return;
    
    pthread_mutex_lock(&s->mutex);
    
    for (int i = 0; i < (int)len; i++) {
        s->iq_buffer[i] = (int16_t)buf[i] - 127;
    }
    s->iq_len = len;
    
    s->data_ready = 1;
    pthread_cond_signal(&s->cond);
    pthread_mutex_unlock(&s->mutex);
}

/* =============================================================================
 * Demodulation Thread
 * =============================================================================
 */

static void *demod_thread_fn(void *arg)
{
    struct arib_state *s = arg;
    int16_t *local_buffer = malloc(DEFAULT_BUF_LENGTH * sizeof(int16_t));
    
    while (!do_exit) {
        pthread_mutex_lock(&s->mutex);
        while (!s->data_ready && !do_exit) {
            pthread_cond_wait(&s->cond, &s->mutex);
        }
        
        if (do_exit) {
            pthread_mutex_unlock(&s->mutex);
            break;
        }
        
        memcpy(local_buffer, s->iq_buffer, s->iq_len * sizeof(int16_t));
        int len = s->iq_len;
        s->data_ready = 0;
        pthread_mutex_unlock(&s->mutex);
        
        process_symbols(s, local_buffer, len);
    }
    
    free(local_buffer);
    return NULL;
}

/* =============================================================================
 * State Initialization
 * =============================================================================
 */

static void state_init(struct arib_state *s)
{
    memset(s, 0, sizeof(*s));
    
    s->channel = ARIB_CALLING_CHANNEL;
    s->gain = -100;
    s->prev_phase = 0.0;
    s->timing_gain = 0.1;
    s->carrier_gain = 0.01;
    
    s->iq_buffer = malloc(DEFAULT_BUF_LENGTH * sizeof(int16_t));
    
    pthread_mutex_init(&s->mutex, NULL);
    pthread_cond_init(&s->cond, NULL);
    
    /* Initialize PN scrambler with default (all ones) */
    pn_init(&s->descrambler, PN_MASK);
    
    generate_rrc_filter(rrc_coeffs, RRC_FILTER_TAPS, ARIB_ROLLOFF, SAMPLES_PER_SYMBOL);
    memset(rrc_delay_i, 0, sizeof(rrc_delay_i));
    memset(rrc_delay_q, 0, sizeof(rrc_delay_q));
}

static void state_cleanup(struct arib_state *s)
{
    free(s->iq_buffer);
    pthread_mutex_destroy(&s->mutex);
    pthread_cond_destroy(&s->cond);
}

/* =============================================================================
 * Main
 * =============================================================================
 */

int main(int argc, char **argv)
{
#ifndef _WIN32
    struct sigaction sigact;
#endif
    int r, opt;
    int dev_given = 0;
    uint32_t direct_freq = 0;
    
    state_init(&state);
    
    while ((opt = getopt(argc, argv, "c:f:d:g:p:TrRvh")) != -1) {
        switch (opt) {
        case 'c':
            state.channel = atoi(optarg);
            if (state.channel < 1 || state.channel > 82) {
                fprintf(stderr, "Invalid channel %d (must be 1-82)\n", state.channel);
                exit(1);
            }
            break;
        case 'f':
            direct_freq = (uint32_t)atof(optarg);
            break;
        case 'd':
            state.dev_index = verbose_device_search(optarg);
            dev_given = 1;
            break;
        case 'g':
            state.gain = (int)(atof(optarg) * 10);
            break;
        case 'p':
            state.ppm_error = atoi(optarg);
            break;
        case 'T':
            state.enable_biastee = 1;
            break;
        case 'r':
            state.raw_output = 1;
            break;
        case 'R':
            state.show_rich = 1;
            break;
        case 'v':
            state.verbose = 1;
            break;
        case 'h':
        default:
            usage();
            break;
        }
    }
    
    if (direct_freq > 0) {
        state.freq = direct_freq;
    } else {
        state.freq = channel_to_freq(state.channel);
    }
    
    if (argc > optind) {
        state.output_file = fopen(argv[optind], "wb");
        if (!state.output_file) {
            fprintf(stderr, "Failed to open output file: %s\n", argv[optind]);
            exit(1);
        }
    } else {
        state.output_file = stdout;
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
    }
    
    if (!dev_given) {
        state.dev_index = verbose_device_search("0");
    }
    if (state.dev_index < 0) {
        exit(1);
    }
    
    r = rtlsdr_open(&state.dev, (uint32_t)state.dev_index);
    if (r < 0) {
        fprintf(stderr, "Failed to open RTL-SDR device #%d\n", state.dev_index);
        exit(1);
    }
    
#ifndef _WIN32
    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGPIPE, &sigact, NULL);
#else
    SetConsoleCtrlHandler((PHANDLER_ROUTINE)sighandler, TRUE);
#endif
    
    if (state.gain == -100) {
        verbose_auto_gain(state.dev);
    } else {
        state.gain = nearest_gain(state.dev, state.gain);
        verbose_gain_set(state.dev, state.gain);
    }
    
    verbose_ppm_set(state.dev, state.ppm_error);
    
    if (state.enable_biastee) {
        rtlsdr_set_bias_tee(state.dev, 1);
        fprintf(stderr, "Bias-T enabled\n");
    }
    
    verbose_set_sample_rate(state.dev, RTL_CAPTURE_RATE);
    verbose_set_frequency(state.dev, state.freq);
    
    fprintf(stderr, "ARIB STD-T98 Demodulator\n");
    fprintf(stderr, "Channel: %d\n", state.channel);
    fprintf(stderr, "Frequency: %.6f MHz\n", state.freq / 1e6);
    fprintf(stderr, "Sample rate: %d Hz\n", RTL_CAPTURE_RATE);
    fprintf(stderr, "Modulation: π/4-QPSK, %d sym/sec\n", ARIB_SYMBOL_RATE);
    fprintf(stderr, "Scrambler: PN(9,5) with User Code\n");
    
    verbose_reset_buffer(state.dev);
    
    pthread_create(&state.demod_thread, NULL, demod_thread_fn, &state);
    
    fprintf(stderr, "Receiving...\n");
    rtlsdr_read_async(state.dev, rtlsdr_callback, &state, 0, DEFAULT_BUF_LENGTH);
    
    do_exit = 1;
    pthread_mutex_lock(&state.mutex);
    state.data_ready = 1;
    pthread_cond_signal(&state.cond);
    pthread_mutex_unlock(&state.mutex);
    
    pthread_join(state.demod_thread, NULL);
    
    fprintf(stderr, "\n=== Statistics ===\n");
    fprintf(stderr, "Frames received:  %lu\n", (unsigned long)state.frames_received);
    fprintf(stderr, "Frames decoded:   %lu\n", (unsigned long)state.frames_decoded);
    fprintf(stderr, "RICH CRC OK:      %lu\n", (unsigned long)state.rich_crc_ok);
    fprintf(stderr, "RICH CRC FAIL:    %lu\n", (unsigned long)state.rich_crc_fail);
    fprintf(stderr, "PICH CRC OK:      %lu\n", (unsigned long)state.pich_crc_ok);
    fprintf(stderr, "PICH CRC FAIL:    %lu\n", (unsigned long)state.pich_crc_fail);
    if (state.user_code_valid) {
        fprintf(stderr, "Last User Code:   0x%03X (%d)\n", 
                state.current_user_code, state.current_user_code);
    }
    
    if (state.output_file != stdout) {
        fclose(state.output_file);
    }
    
    rtlsdr_close(state.dev);
    state_cleanup(&state);
    
    return 0;
}

/* vim: tabstop=4:softtabstop=4:shiftwidth=4:expandtab */
