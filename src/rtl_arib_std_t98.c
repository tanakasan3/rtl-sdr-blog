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
#define CRC_POLY                0x43        /* CRC-6: G(x) = 1 + x + x^6 (0x43 = 1000011) */
#define CONV_K                  6           /* Convolutional code constraint length */
#define CONV_RATE               2           /* 1/2 rate */
#define CONV_G1                 0x35        /* Generator polynomial 1 */
#define CONV_G2                 0x3D        /* Generator polynomial 2 */

/* PN scrambler */
#define PN_POLY_TAP1            9
#define PN_POLY_TAP2            5

/* Channel frequencies for 351 MHz registered stations */
#define ARIB_BASE_FREQ          351200000   /* Channel 1 base frequency */
#define ARIB_CALLING_CHANNEL    15          /* 351.28750 MHz */

/* Buffer sizes */
#define DEFAULT_BUF_LENGTH      (16 * 16384)
#define RRC_FILTER_TAPS         65          /* RRC filter length */
#define MAX_FRAME_BUFFER        4096

/* π/4-QPSK constellation points (normalized) */
/* Even symbols: 0, π/2, π, 3π/2 */
/* Odd symbols: π/4, 3π/4, 5π/4, 7π/4 */
static const double PI_4_QPSK_EVEN_I[4] = { 1.0,  0.0, -1.0,  0.0 };
static const double PI_4_QPSK_EVEN_Q[4] = { 0.0,  1.0,  0.0, -1.0 };
static const double PI_4_QPSK_ODD_I[4]  = { 0.7071,  -0.7071, -0.7071,  0.7071 };
static const double PI_4_QPSK_ODD_Q[4]  = { 0.7071,   0.7071, -0.7071, -0.7071 };

/* Differential encoding mapping for π/4-QPSK */
/* Input dibits (MSB first): 00, 01, 11, 10 -> phase changes: π/4, 3π/4, -3π/4, -π/4 */
static const double DIFF_PHASE_MAP[4] = { M_PI/4, 3*M_PI/4, -3*M_PI/4, -M_PI/4 };

static volatile int do_exit = 0;

/* Root Raised Cosine filter coefficients */
static double rrc_coeffs[RRC_FILTER_TAPS];
static double rrc_delay_i[RRC_FILTER_TAPS];
static double rrc_delay_q[RRC_FILTER_TAPS];

/* Structures */
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
    int frame_type;  /* 0 = SC, 1 = SB0 */
    
    /* PN descrambler state */
    uint16_t pn_shift_reg;
    
    /* Statistics */
    uint64_t frames_received;
    uint64_t sync_errors;
    uint64_t crc_errors;
    
    /* Output */
    FILE *output_file;
    int raw_output;
    int verbose;
    
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
        "\t[-r]           Raw symbol output (no decoding)\n"
        "\t[-v]           Verbose output\n"
        "\t[-h]           Show this help\n\n"
        "Output: Decoded frames written to stdout or specified file\n\n"
        "Examples:\n"
        "\trtl_arib_std_t98 -c 15              # Monitor calling channel\n"
        "\trtl_arib_std_t98 -c 15 -v output.bin\n"
        "\trtl_arib_std_t98 -f 351287500       # Direct frequency\n\n"
        "Technical Specs (ARIB STD-T98):\n"
        "\tModulation:    π/4-QPSK\n"
        "\tSymbol Rate:   4,800 sym/sec\n"
        "\tBit Rate:      9,600 bps\n"
        "\tFrame Length:  40 ms (384 bits)\n"
        "\tRoll-off:      α=0.2\n\n"
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

/*
 * Generate Root Raised Cosine filter coefficients
 * α = roll-off factor (0.2 for ARIB STD-T98)
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
            /* t = 0 */
            h = (1.0 - alpha + 4.0 * alpha / M_PI);
        } else if (fabs(fabs(t) - T / (4.0 * alpha)) < 1e-10) {
            /* t = ±T/(4α) */
            h = (alpha / sqrt(2.0)) * (
                (1.0 + 2.0/M_PI) * sin(M_PI / (4.0 * alpha)) +
                (1.0 - 2.0/M_PI) * cos(M_PI / (4.0 * alpha))
            );
        } else {
            /* General case */
            double sinc_term = sin(M_PI * t * (1.0 - alpha)) + 
                               4.0 * alpha * t * cos(M_PI * t * (1.0 + alpha));
            double denom = M_PI * t * (1.0 - pow(4.0 * alpha * t, 2));
            h = sinc_term / denom;
        }
        
        coeffs[i] = h;
        sum += h;
    }
    
    /* Normalize */
    for (int i = 0; i < num_taps; i++) {
        coeffs[i] /= sum;
    }
}

/*
 * Apply RRC filter to IQ samples
 */
static void rrc_filter(double *in_i, double *in_q, double *out_i, double *out_q)
{
    /* Shift delay lines */
    memmove(&rrc_delay_i[1], &rrc_delay_i[0], (RRC_FILTER_TAPS - 1) * sizeof(double));
    memmove(&rrc_delay_q[1], &rrc_delay_q[0], (RRC_FILTER_TAPS - 1) * sizeof(double));
    
    rrc_delay_i[0] = *in_i;
    rrc_delay_q[0] = *in_q;
    
    /* Convolve */
    *out_i = 0.0;
    *out_q = 0.0;
    for (int i = 0; i < RRC_FILTER_TAPS; i++) {
        *out_i += rrc_delay_i[i] * rrc_coeffs[i];
        *out_q += rrc_delay_q[i] * rrc_coeffs[i];
    }
}

/*
 * Calculate channel frequency from channel number
 */
static uint32_t channel_to_freq(int channel)
{
    if (channel >= 1 && channel <= 70) {
        /* Channels 1-70: 351.20000 - 351.63125 MHz */
        return 351200000 + (channel - 1) * ARIB_CHANNEL_SPACING;
    } else if (channel >= 71 && channel <= 82) {
        /* Channels 71-82: 351.03125 - 351.10000 MHz */
        return 351031250 + (channel - 71) * ARIB_CHANNEL_SPACING;
    }
    return 351287500; /* Default to calling channel 15 */
}

/*
 * π/4-QPSK differential demodulation
 * Returns 2-bit dibit (MSB first)
 */
static int pi4_qpsk_demod(double i_sample, double q_sample, double *prev_phase)
{
    double phase = atan2(q_sample, i_sample);
    double delta_phase = phase - *prev_phase;
    
    /* Normalize to [-π, π] */
    while (delta_phase > M_PI) delta_phase -= 2.0 * M_PI;
    while (delta_phase < -M_PI) delta_phase += 2.0 * M_PI;
    
    *prev_phase = phase;
    
    /* Map phase change to dibit */
    /* π/4-QPSK differential mapping:
     * Δφ ≈ π/4   -> 00
     * Δφ ≈ 3π/4  -> 01
     * Δφ ≈ -3π/4 -> 11
     * Δφ ≈ -π/4  -> 10
     */
    int dibit;
    if (delta_phase >= 0 && delta_phase < M_PI/2) {
        dibit = 0b00;  /* π/4 region */
    } else if (delta_phase >= M_PI/2 && delta_phase <= M_PI) {
        dibit = 0b01;  /* 3π/4 region */
    } else if (delta_phase < 0 && delta_phase >= -M_PI/2) {
        dibit = 0b10;  /* -π/4 region */
    } else {
        dibit = 0b11;  /* -3π/4 region */
    }
    
    return dibit;
}

/*
 * Gardner timing error detector for symbol timing recovery
 */
static double gardner_ted(double early_i, double early_q, 
                          double on_time_i, double on_time_q,
                          double late_i, double late_q)
{
    double error_i = (late_i - early_i) * on_time_i;
    double error_q = (late_q - early_q) * on_time_q;
    return error_i + error_q;
}

/*
 * PN(9,5) descrambler
 * Polynomial: x^9 + x^5 + 1
 */
static uint8_t pn_descramble_bit(struct arib_state *s, uint8_t bit)
{
    /* Generate PN bit */
    uint8_t pn_bit = ((s->pn_shift_reg >> (PN_POLY_TAP1 - 1)) ^ 
                      (s->pn_shift_reg >> (PN_POLY_TAP2 - 1))) & 1;
    
    /* Shift register */
    s->pn_shift_reg = (s->pn_shift_reg << 1) | pn_bit;
    s->pn_shift_reg &= 0x1FF; /* Keep 9 bits */
    
    /* XOR with input bit */
    return bit ^ pn_bit;
}

/*
 * Reset PN scrambler to initial state
 */
static void pn_reset(struct arib_state *s)
{
    s->pn_shift_reg = 0x1FF; /* All ones initial state */
}

/*
 * Check for sync word in bit buffer
 */
static int check_sync(struct arib_state *s)
{
    /* Check Service Channel sync (20 bits) */
    uint32_t sc_mask = (1UL << SYNC_SC_BITS) - 1;
    if ((s->bit_buffer & sc_mask) == SYNC_SC_PATTERN) {
        return 1; /* SC sync found */
    }
    
    /* Check Sync Burst sync (32 bits) */
    uint32_t sb0_mask = (1ULL << SYNC_SB0_BITS) - 1;
    if ((s->bit_buffer & sb0_mask) == SYNC_SB0_PATTERN) {
        return 2; /* SB0 sync found */
    }
    
    return 0;
}

/*
 * CRC-6 calculation
 * G(x) = 1 + x + x^6
 */
static uint8_t crc6_calc(uint8_t *data, int num_bits)
{
    uint8_t crc = 0;
    
    for (int i = 0; i < num_bits; i++) {
        uint8_t bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        uint8_t msb = (crc >> 5) & 1;
        crc = ((crc << 1) | bit) & 0x3F;
        if (msb) {
            crc ^= 0x03; /* x + 1 */
        }
    }
    
    return crc;
}

/*
 * Viterbi decoder for rate 1/2, K=6 convolutional code
 * Simple hard-decision implementation
 */
struct viterbi_state {
    uint32_t path_metric[32];  /* 2^(K-1) states */
    uint32_t path[32];
    int len;
};

static void viterbi_init(struct viterbi_state *v)
{
    memset(v, 0, sizeof(*v));
    for (int i = 1; i < 32; i++) {
        v->path_metric[i] = 0xFFFFFFFF;
    }
}

static int hamming_weight(uint8_t x)
{
    int w = 0;
    while (x) { w += x & 1; x >>= 1; }
    return w;
}

static void viterbi_decode_pair(struct viterbi_state *v, uint8_t sym0, uint8_t sym1)
{
    uint32_t new_metric[32];
    uint32_t new_path[32];
    
    memcpy(new_metric, v->path_metric, sizeof(new_metric));
    memcpy(new_path, v->path, sizeof(new_path));
    
    for (int state = 0; state < 32; state++) {
        for (int input = 0; input < 2; input++) {
            int next_state = ((state << 1) | input) & 0x1F;
            
            /* Calculate expected output */
            int reg = (state << 1) | input;
            uint8_t exp0 = __builtin_popcount(reg & CONV_G1) & 1;
            uint8_t exp1 = __builtin_popcount(reg & CONV_G2) & 1;
            
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

static int viterbi_finish(struct viterbi_state *v, uint8_t *output, int max_bits)
{
    /* Find best ending state */
    int best_state = 0;
    uint32_t best_metric = v->path_metric[0];
    for (int i = 1; i < 32; i++) {
        if (v->path_metric[i] < best_metric) {
            best_metric = v->path_metric[i];
            best_state = i;
        }
    }
    
    /* Traceback */
    int num_bits = v->len < max_bits ? v->len : max_bits;
    for (int i = 0; i < num_bits; i++) {
        output[num_bits - 1 - i] = (v->path[best_state] >> i) & 1;
    }
    
    return num_bits;
}

/*
 * Block deinterleaver
 */
static void deinterleave(uint8_t *input, uint8_t *output, int rows, int cols)
{
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            output[r * cols + c] = input[c * rows + r];
        }
    }
}

/*
 * Decode a received frame
 */
static void decode_frame(struct arib_state *s)
{
    if (s->verbose) {
        fprintf(stderr, "Frame %lu: Type=%s, Bits=%d\n",
                (unsigned long)s->frames_received,
                s->frame_type == 1 ? "SC" : "SB0",
                s->frame_bit_idx);
    }
    
    /* For raw output, just dump the bits */
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
    }
    
    /* TODO: Full frame decoding with:
     * - Deinterleaving
     * - Viterbi decoding
     * - CRC checking
     * - RICH/PICH extraction
     * - Call sign decoding
     */
    
    s->frames_received++;
}

/*
 * Process received symbols
 */
static void process_symbols(struct arib_state *s, int16_t *samples, int len)
{
    for (int i = 0; i < len - 1; i += 2) {
        /* Convert to double and normalize */
        double i_raw = (double)samples[i] / 32768.0;
        double q_raw = (double)samples[i + 1] / 32768.0;
        
        /* Apply RRC matched filter */
        double i_filt, q_filt;
        rrc_filter(&i_raw, &q_raw, &i_filt, &q_filt);
        
        s->samples_since_symbol++;
        
        /* Simple symbol timing - sample at peak */
        if (s->samples_since_symbol >= SAMPLES_PER_SYMBOL) {
            s->samples_since_symbol = 0;
            
            /* Demodulate symbol */
            int dibit = pi4_qpsk_demod(i_filt, q_filt, &s->prev_phase);
            
            /* Extract bits (MSB first) */
            uint8_t bit1 = (dibit >> 1) & 1;
            uint8_t bit0 = dibit & 1;
            
            /* Add to bit buffer for sync detection */
            s->bit_buffer = (s->bit_buffer << 2) | dibit;
            s->bit_count += 2;
            
            /* Check for sync if not already synced */
            if (!s->frame_synced) {
                int sync_type = check_sync(s);
                if (sync_type) {
                    s->frame_synced = 1;
                    s->frame_type = sync_type;
                    s->frame_bit_idx = 0;
                    pn_reset(s);
                    
                    if (s->verbose) {
                        fprintf(stderr, "SYNC: %s at symbol %d\n",
                                sync_type == 1 ? "Service Channel" : "Sync Burst",
                                s->symbol_count);
                    }
                }
            } else {
                /* Store frame bits */
                if (s->frame_bit_idx < ARIB_FRAME_BITS) {
                    s->frame_bits[s->frame_bit_idx++] = bit1;
                }
                if (s->frame_bit_idx < ARIB_FRAME_BITS) {
                    s->frame_bits[s->frame_bit_idx++] = bit0;
                }
                
                /* Check for frame completion */
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

/*
 * RTL-SDR callback
 */
static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    struct arib_state *s = ctx;
    
    if (do_exit) return;
    if (!ctx) return;
    
    pthread_mutex_lock(&s->mutex);
    
    /* Convert uint8 to int16 */
    int samples = len / 2;
    for (int i = 0; i < (int)len; i++) {
        s->iq_buffer[i] = (int16_t)buf[i] - 127;
    }
    s->iq_len = len;
    
    s->data_ready = 1;
    pthread_cond_signal(&s->cond);
    pthread_mutex_unlock(&s->mutex);
}

/*
 * Demodulation thread
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
        
        /* Process samples */
        process_symbols(s, local_buffer, len);
    }
    
    free(local_buffer);
    return NULL;
}

/*
 * Initialize state
 */
static void state_init(struct arib_state *s)
{
    memset(s, 0, sizeof(*s));
    
    s->channel = ARIB_CALLING_CHANNEL;
    s->gain = -100; /* Auto gain */
    s->prev_phase = 0.0;
    s->timing_gain = 0.1;
    s->carrier_gain = 0.01;
    
    s->iq_buffer = malloc(DEFAULT_BUF_LENGTH * sizeof(int16_t));
    
    pthread_mutex_init(&s->mutex, NULL);
    pthread_cond_init(&s->cond, NULL);
    
    /* Initialize RRC filter */
    generate_rrc_filter(rrc_coeffs, RRC_FILTER_TAPS, ARIB_ROLLOFF, SAMPLES_PER_SYMBOL);
    memset(rrc_delay_i, 0, sizeof(rrc_delay_i));
    memset(rrc_delay_q, 0, sizeof(rrc_delay_q));
}

/*
 * Cleanup state
 */
static void state_cleanup(struct arib_state *s)
{
    free(s->iq_buffer);
    pthread_mutex_destroy(&s->mutex);
    pthread_cond_destroy(&s->cond);
}

int main(int argc, char **argv)
{
#ifndef _WIN32
    struct sigaction sigact;
#endif
    int r, opt;
    int dev_given = 0;
    uint32_t direct_freq = 0;
    
    state_init(&state);
    
    while ((opt = getopt(argc, argv, "c:f:d:g:p:Trvh")) != -1) {
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
        case 'v':
            state.verbose = 1;
            break;
        case 'h':
        default:
            usage();
            break;
        }
    }
    
    /* Set frequency */
    if (direct_freq > 0) {
        state.freq = direct_freq;
    } else {
        state.freq = channel_to_freq(state.channel);
    }
    
    /* Open output file */
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
    
    /* Find device */
    if (!dev_given) {
        state.dev_index = verbose_device_search("0");
    }
    if (state.dev_index < 0) {
        exit(1);
    }
    
    /* Open device */
    r = rtlsdr_open(&state.dev, (uint32_t)state.dev_index);
    if (r < 0) {
        fprintf(stderr, "Failed to open RTL-SDR device #%d\n", state.dev_index);
        exit(1);
    }
    
    /* Setup signal handlers */
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
    
    /* Configure device */
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
    
    /* Set sample rate */
    verbose_set_sample_rate(state.dev, RTL_CAPTURE_RATE);
    
    /* Set frequency */
    verbose_set_frequency(state.dev, state.freq);
    
    fprintf(stderr, "ARIB STD-T98 Demodulator\n");
    fprintf(stderr, "Channel: %d\n", state.channel);
    fprintf(stderr, "Frequency: %.6f MHz\n", state.freq / 1e6);
    fprintf(stderr, "Sample rate: %d Hz\n", RTL_CAPTURE_RATE);
    fprintf(stderr, "Modulation: π/4-QPSK, %d sym/sec, %d bps\n", 
            ARIB_SYMBOL_RATE, ARIB_BIT_RATE);
    
    /* Reset buffer */
    verbose_reset_buffer(state.dev);
    
    /* Start demodulation thread */
    pthread_create(&state.demod_thread, NULL, demod_thread_fn, &state);
    
    /* Start async reading */
    fprintf(stderr, "Receiving...\n");
    rtlsdr_read_async(state.dev, rtlsdr_callback, &state, 0, DEFAULT_BUF_LENGTH);
    
    /* Cleanup */
    do_exit = 1;
    pthread_mutex_lock(&state.mutex);
    state.data_ready = 1;
    pthread_cond_signal(&state.cond);
    pthread_mutex_unlock(&state.mutex);
    
    pthread_join(state.demod_thread, NULL);
    
    fprintf(stderr, "\nStatistics:\n");
    fprintf(stderr, "  Frames received: %lu\n", (unsigned long)state.frames_received);
    fprintf(stderr, "  Sync errors: %lu\n", (unsigned long)state.sync_errors);
    
    if (state.output_file != stdout) {
        fclose(state.output_file);
    }
    
    rtlsdr_close(state.dev);
    state_cleanup(&state);
    
    return 0;
}

/* vim: tabstop=4:softtabstop=4:shiftwidth=4:expandtab */
