/*
 * lpc2wav.c -- TMS5xxx LPC speech synthesizer decoder
 *
 * Decodes TMS5100/5110/5200/5220 LPC bitstreams to 16-bit mono 8 kHz WAV.
 *
 * Based on pyti_lpc_cmd (Copyright (C) 2026 Kris Kirby, KE4AHR) and the
 * tms5xxx_cleanroom_specification.md.  No GPL source code other than that
 * project was referenced.
 *
 * Usage:
 *   lpc2wav -infile input.lpc -outfile output.wav [-chip tms5220] [-gain 100]
 *           [-startaddr 0x0000] [-full-interp] [-nointerp] [-pulse]
 *           [-maxframes 200] [-verbose]
 *
 * Chips: tms5100, tms5110, tms5200, tms5220 (default: tms5220)
 * Gain:  0-300 percent (default: 100)
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* =========================================================================
 * Chip tables
 * ========================================================================= */

/* --- TMS5220 ------------------------------------------------------------ */
static const int8_t CHIRP_5220[52] = {
    0x00,0x03,0x0f,0x28,0x4c,0x6c,0x71,0x50,
    0x25,0x26,0x4c,0x44,0x1a,0x32,0x3b,0x13,
    0x37,0x1a,0x25,0x1f,0x1d,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};
static const int ENERGY_5220[16] = {0,1,2,3,4,6,8,11,16,23,33,47,63,85,114,0};
static const int PITCH_5220[64] = {
     0,15,16,17,18,19,20,21,22,23,
    24,25,26,27,28,29,30,31,32,33,
    34,35,36,37,38,39,40,41,42,44,
    46,48,50,52,53,56,58,60,62,65,
    68,70,72,76,78,80,84,86,91,94,
    98,101,105,109,114,118,122,127,132,137,
    142,148,153,159,
};
static const int K0_5220[32] = {
    -501,-498,-497,-495,-493,-491,-488,-482,
    -478,-474,-469,-464,-459,-452,-445,-437,
    -412,-380,-339,-288,-227,-158,-81,-1,
     80,157,226,287,337,379,411,436,
};
static const int K1_5220[32] = {
    -328,-303,-274,-244,-211,-175,-138,-99,
     -59,-18,24,64,105,143,180,215,
     248,278,306,331,354,374,392,408,
     422,435,445,455,463,470,476,506,
};
static const int K2_5220[16] = {
    -441,-387,-333,-279,-225,-171,-117,-63,
      -9,45,98,152,206,260,314,368,
};
static const int K3_5220[16] = {
    -328,-273,-217,-161,-106,-50,5,61,
     116,172,228,283,339,394,450,506,
};
static const int K4_5220[16] = {
    -328,-282,-235,-189,-142,-96,-50,-3,
      43,90,136,182,229,275,322,368,
};
static const int K5_5220[16] = {
    -256,-212,-168,-123,-79,-35,10,54,
      98,143,187,232,276,320,365,409,
};
static const int K6_5220[16] = {
    -308,-260,-212,-164,-117,-69,-21,27,
      75,122,170,218,266,314,361,409,
};
static const int K7_5220[8]  = {-256,-161,-66,29,124,219,314,409};
static const int K8_5220[8]  = {-256,-176,-96,-15,65,146,226,307};
static const int K9_5220[8]  = {-205,-132,-59,14,87,160,234,307};

/* --- TMS5200 ------------------------------------------------------------ */
static const int8_t CHIRP_5200[52] = { /* same waveform as 5220 */
    0x00,0x03,0x0f,0x28,0x4c,0x6c,0x71,0x50,
    0x25,0x26,0x4c,0x44,0x1a,0x32,0x3b,0x13,
    0x37,0x1a,0x25,0x1f,0x1d,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};
static const int ENERGY_5200[16] = {0,1,2,3,4,6,8,11,16,23,33,47,63,85,114,0};
static const int PITCH_5200[64] = {
     0,14,15,16,17,18,19,20,21,22,
    23,24,25,26,27,28,29,30,31,32,
    34,36,38,40,41,43,45,48,49,51,
    54,55,57,60,62,64,68,72,74,76,
    81,85,87,90,96,99,103,107,112,117,
    122,127,133,139,145,151,157,164,171,178,
    186,194,202,211,
};
static const int K0_5200[32] = {
    -501,-498,-495,-490,-485,-478,-469,-459,
    -446,-431,-412,-389,-362,-331,-295,-253,
    -207,-156,-102,-45,13,70,126,179,
     228,272,311,345,374,399,420,437,
};
static const int K1_5200[32] = {
    -376,-357,-335,-312,-286,-258,-227,-195,
    -161,-124,-87,-49,-10,29,68,106,
     143,178,212,243,272,299,324,346,
     366,384,400,414,427,438,448,506,
};
static const int K2_5200[16] = {
    -407,-381,-349,-311,-268,-218,-162,-102,
     -39,25,89,149,206,257,302,341,
};
static const int K3_5200[16] = {
    -290,-252,-209,-163,-114,-62,-9,44,
      97,147,194,238,278,313,344,371,
};
static const int K4_5200[16] = {
    -318,-283,-245,-202,-156,-107,-56,-3,
      49,101,150,196,239,278,313,344,
};
static const int K5_5200[16] = {
    -193,-152,-109,-65,-20,26,71,115,
     158,198,235,270,301,330,355,377,
};
static const int K6_5200[16] = {
    -254,-218,-180,-140,-97,-53,-8,36,
      81,124,165,204,240,274,304,332,
};
static const int K7_5200[8]  = {-205,-112,-10,92,187,269,336,387};
static const int K8_5200[8]  = {-249,-183,-110,-32,48,126,198,261};
static const int K9_5200[8]  = {-190,-133,-73,-10,53,115,173,227};

/* --- TMS5110 ------------------------------------------------------------ */
static const int8_t CHIRP_5110[32] = {
     0,16,18,19,21,24,26,28,31,35,
    37,42,44,47,50,53,56,59,63,67,
    71,75,79,84,89,94,100,106,112,126,
    141,150,
};
static const int ENERGY_5110[16] = {0,1,2,3,4,6,8,11,16,23,33,47,63,85,114,0};
static const int PITCH_5110[32] = {
     0,15,16,17,19,21,22,25,26,29,
    32,36,40,42,46,50,55,60,64,68,
    72,76,80,84,86,93,101,110,120,132,
    144,159,
};
/* k0-k9 for 5110 are identical to 5220 */
#define K0_5110 K0_5220
#define K1_5110 K1_5220
#define K2_5110 K2_5220
#define K3_5110 K3_5220
#define K4_5110 K4_5220
#define K5_5110 K5_5220
#define K6_5110 K6_5220
#define K7_5110 K7_5220
#define K8_5110 K8_5220
#define K9_5110 K9_5220

/* --- TMS5100 ------------------------------------------------------------ */
static const int8_t CHIRP_5100[50] = {
     0,42,212,50,178,18,37,20,2,225,
    197,2,95,90,5,15,38,252,165,165,
    214,221,220,252,37,43,34,33,15,255,
    248,238,237,239,247,246,250,0,3,2,
      1,0,0,0,0,0,0,0,0,0,
};
static const int ENERGY_5100[16] = {0,0,1,1,2,3,5,7,10,15,21,30,43,61,86,0};
static const int PITCH_5100[32] = {
     0,41,43,45,47,49,51,53,55,58,
    60,63,66,70,73,76,79,83,87,90,
    94,99,103,107,112,118,123,129,134,140,
    147,153,
};
static const int K0_5100[32] = {
    -501,-497,-493,-488,-480,-471,-460,-446,
    -427,-405,-378,-344,-305,-259,-206,-148,
     -86,-21,45,110,171,227,277,320,
     357,388,413,434,451,464,474,498,
};
static const int K1_5100[32] = {
    -349,-328,-305,-280,-252,-223,-192,-158,
    -124,-88,-51,-14,23,60,97,133,
     167,199,230,259,286,310,333,354,
     372,389,404,417,429,439,449,506,
};
static const int K2_5100[16] = {
    -397,-365,-327,-282,-229,-170,-104,-36,
      35,104,169,228,281,326,364,396,
};
static const int K3_5100[16] = {
    -369,-334,-293,-245,-191,-131,-67,-1,
      64,128,188,243,291,332,367,397,
};
static const int K4_5100[16] = {
    -319,-286,-250,-211,-168,-122,-74,-25,
      24,73,121,167,210,249,285,318,
};
static const int K5_5100[16] = {
    -290,-252,-209,-163,-114,-62,-9,44,
      97,147,194,238,278,313,344,371,
};
static const int K6_5100[16] = {
    -291,-256,-216,-174,-128,-80,-31,19,
      69,117,163,206,246,283,316,345,
};
static const int K7_5100[8]  = {-218,-133,-38,59,152,235,305,361};
static const int K8_5100[8]  = {-226,-157,-82,-3,76,151,220,280};
static const int K9_5100[8]  = {-179,-122,-61,1,62,123,179,231};

/* =========================================================================
 * Chip descriptor
 * ========================================================================= */
typedef struct {
    const char     *name;
    int             pitch_count;   /* 32 or 64 */
    int             chirp_len;
    const int8_t   *chirp;
    const int      *energy;        /* 16 entries */
    const int      *pitch;         /* 32 or 64 entries */
    const int      *k[10];         /* k0..k9 */
    int             k_len[10];     /* entry count per table */
} ChipDef;

static const ChipDef CHIPS[4] = {
    {
        "tms5100", 32, 50, CHIRP_5100,
        ENERGY_5100, PITCH_5100,
        { K0_5100,K1_5100,K2_5100,K3_5100,K4_5100,
          K5_5100,K6_5100,K7_5100,K8_5100,K9_5100 },
        { 32,32,16,16,16,16,16,8,8,8 },
    },
    {
        "tms5110", 32, 32, CHIRP_5110,
        ENERGY_5110, PITCH_5110,
        { K0_5110,K1_5110,K2_5110,K3_5110,K4_5110,
          K5_5110,K6_5110,K7_5110,K8_5110,K9_5110 },
        { 32,32,16,16,16,16,16,8,8,8 },
    },
    {
        "tms5200", 64, 52, CHIRP_5200,
        ENERGY_5200, PITCH_5200,
        { K0_5200,K1_5200,K2_5200,K3_5200,K4_5200,
          K5_5200,K6_5200,K7_5200,K8_5200,K9_5200 },
        { 32,32,16,16,16,16,16,8,8,8 },
    },
    {
        "tms5220", 64, 52, CHIRP_5220,
        ENERGY_5220, PITCH_5220,
        { K0_5220,K1_5220,K2_5220,K3_5220,K4_5220,
          K5_5220,K6_5220,K7_5220,K8_5220,K9_5220 },
        { 32,32,16,16,16,16,16,8,8,8 },
    },
};
#define NUM_CHIPS 4

static const ChipDef *find_chip(const char *name)
{
    int i;
    for (i = 0; i < NUM_CHIPS; i++)
        if (strcmp(CHIPS[i].name, name) == 0)
            return &CHIPS[i];
    return NULL;
}

/* =========================================================================
 * Bitstream reader
 * ========================================================================= */
typedef struct {
    const uint8_t *data;
    size_t         len;
    size_t         byte_pos;
    int            bit_pos;   /* 0-7 */
} BitstreamReader;

static void bsr_init(BitstreamReader *r, const uint8_t *data, size_t len)
{
    r->data     = data;
    r->len      = len;
    r->byte_pos = 0;
    r->bit_pos  = 0;
}

static uint8_t bsr_get_byte(const BitstreamReader *r, int offset)
{
    size_t idx = r->byte_pos + (size_t)offset;
    uint8_t b;
    if (idx >= r->len) return 0;
    b = r->data[idx];
    /* Bit-reverse each byte (spec sec. 3.1) */
    b = (uint8_t)(((b >> 4) | (b << 4)) & 0xFF);
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

static int bsr_get_bits(BitstreamReader *r, int count)
{
    uint16_t w = (uint16_t)(bsr_get_byte(r, 0) << 8);
    int value;
    if (r->bit_pos + count > 8)
        w |= bsr_get_byte(r, 1);
    w = (uint16_t)((w << r->bit_pos) & 0xFFFF);
    value = (int)(w >> (16 - count));
    r->bit_pos += count;
    if (r->bit_pos >= 8) {
        r->bit_pos -= 8;
        r->byte_pos++;
    }
    return value;
}

/* =========================================================================
 * Frame decoder
 * ========================================================================= */
typedef struct {
    int  energy_idx;
    int  energy;
    int  period;
    int  repeat;
    int  k[10];
    int  is_silence;
    int  is_stop;
} LPCFrame;

static LPCFrame decode_frame(BitstreamReader *r, const ChipDef *chip, const int prev_k[10])
{
    LPCFrame f;
    int pitch_bits, pitch_idx;
    int i;
    memset(&f, 0, sizeof(f));

    f.energy_idx = bsr_get_bits(r, 4);

    if (f.energy_idx == 0) {
        f.is_silence = 1;
        return f;
    }
    if (f.energy_idx == 0xF) {
        f.is_stop = 1;
        return f;
    }

    f.energy = chip->energy[f.energy_idx];
    f.repeat = bsr_get_bits(r, 1);

    pitch_bits = (chip->pitch_count == 64) ? 6 : 5;
    pitch_idx  = bsr_get_bits(r, pitch_bits);
    f.period   = chip->pitch[pitch_idx];

    if (f.repeat) {
        for (i = 0; i < 10; i++) f.k[i] = prev_k[i];
        return f;
    }

    /* k0, k1: 5-bit */
    f.k[0] = chip->k[0][bsr_get_bits(r, 5)];
    f.k[1] = chip->k[1][bsr_get_bits(r, 5)];
    /* k2, k3: 4-bit */
    f.k[2] = chip->k[2][bsr_get_bits(r, 4)];
    f.k[3] = chip->k[3][bsr_get_bits(r, 4)];

    if (f.period != 0) {
        /* Voiced: k4-k9 */
        f.k[4] = chip->k[4][bsr_get_bits(r, 4)];
        f.k[5] = chip->k[5][bsr_get_bits(r, 4)];
        f.k[6] = chip->k[6][bsr_get_bits(r, 4)];
        f.k[7] = chip->k[7][bsr_get_bits(r, 3)];
        f.k[8] = chip->k[8][bsr_get_bits(r, 3)];
        f.k[9] = chip->k[9][bsr_get_bits(r, 3)];
    }
    /* Unvoiced: k4-k9 remain 0 */

    return f;
}

/* =========================================================================
 * Synthesizer
 * ========================================================================= */
#define SAMPLES_PER_FRAME 200
#define K_SCALE           512.0
#define OUTPUT_GAIN       1.5

typedef struct {
    double  x[10];
    unsigned int synth_rand;
    int     period_counter;
} SynthState;

static void synth_init(SynthState *s)
{
    int i;
    for (i = 0; i < 10; i++) s->x[i] = 0.0;
    s->synth_rand     = 1;
    s->period_counter = 0;
}

static double excitation(SynthState *s, int energy, int period, const int8_t *chirp, int chirp_len, int use_pulse)
{
    double u10;
    if (period > 0) {
        int sv;
        if (use_pulse)
            sv = (s->period_counter == 0) ? 127 : 0;
        else
            sv = (s->period_counter < chirp_len && s->period_counter < 41)
                 ? chirp[s->period_counter] : 0;
        u10 = (sv / 256.0) * (energy / 256.0);
        if (s->period_counter >= period - 1)
            s->period_counter = 0;
        else
            s->period_counter++;
    } else {
        s->synth_rand = ((s->synth_rand >> 1) ^ ((s->synth_rand & 1) ? 0xB800 : 0)) & 0xFFFF;
        /* Hardware fires ±0x40 (±64) for unvoiced, same energy scaling as voiced */
        int noise = (s->synth_rand & 1) ? 64 : -64;
        u10 = (noise / 256.0) * (energy / 256.0);
    }
    return u10;
}

static double lattice_filter(SynthState *s, double u10, const int k[10])
{
    double *x = s->x;
    double u9,u8,u7,u6,u5,u4,u3,u2,u1,u0;

    /* Forward path */
    u9  = u10 - (k[9] / K_SCALE) * x[9];
    u8  = u9  - (k[8] / K_SCALE) * x[8];
    u7  = u8  - (k[7] / K_SCALE) * x[7];
    u6  = u7  - (k[6] / K_SCALE) * x[6];
    u5  = u6  - (k[5] / K_SCALE) * x[5];
    u4  = u5  - (k[4] / K_SCALE) * x[4];
    u3  = u4  - (k[3] / K_SCALE) * x[3];
    u2  = u3  - (k[2] / K_SCALE) * x[2];
    u1  = u2  - (k[1] / K_SCALE) * x[1];
    u0  = u1  - (k[0] / K_SCALE) * x[0];

    if (u0 >  1.0) u0 =  1.0;
    if (u0 < -1.0) u0 = -1.0;

    /* Reverse path: update delay states */
    x[9] = x[8] + (k[8] / K_SCALE) * u8;
    x[8] = x[7] + (k[7] / K_SCALE) * u7;
    x[7] = x[6] + (k[6] / K_SCALE) * u6;
    x[6] = x[5] + (k[5] / K_SCALE) * u5;
    x[5] = x[4] + (k[4] / K_SCALE) * u4;
    x[4] = x[3] + (k[3] / K_SCALE) * u3;
    x[3] = x[2] + (k[2] / K_SCALE) * u2;
    x[2] = x[1] + (k[1] / K_SCALE) * u1;
    x[1] = x[0] + (k[0] / K_SCALE) * u0;
    x[0] = u0;

    return u0;
}

/* Returns a heap-allocated array of float samples; caller must free().
   *out_count receives the sample count. */
static double *synthesize(
    const uint8_t *data, size_t datalen,
    const ChipDef *chip,
    int use_interp, int unvoiced_nointerp, int use_pulse, int max_frames, int verbose,
    size_t *out_count)
{
    BitstreamReader r;
    SynthState      s;
    size_t cap = 8000 * 10;   /* 10 seconds initial capacity */
    double *samples = (double *)malloc(cap * sizeof(double));
    size_t  nsamples = 0;

    int prev_k[10] = {0};
    int cur_energy = 0, cur_period = 0, cur_k[10] = {0};
    int ending_countdown = -1;
    int frame_count = 0;
    int i, s_idx;

    bsr_init(&r, data, datalen);
    synth_init(&s);

    while (1) {
        LPCFrame frame;

        /* Decode or fabricate */
        if (ending_countdown < 0) {
            frame = decode_frame(&r, chip, prev_k);
        } else {
            memset(&frame, 0, sizeof(frame));
            frame.is_silence = 1;
        }

        /* Handle stop frame */
        if (frame.is_stop && ending_countdown < 0) {
            ending_countdown = 2;
            memset(&frame, 0, sizeof(frame));
            frame.is_silence = 1;
        }

        int from_energy = cur_energy;
        int from_period = cur_period;
        int from_k[10];
        for (i = 0; i < 10; i++) from_k[i] = cur_k[i];

        int tgt_energy = frame.energy;
        int tgt_period = frame.period;
        int tgt_k[10];
        for (i = 0; i < 10; i++) tgt_k[i] = frame.k[i];

        int now_voiced  = (cur_period != 0);
        int now_silence = (cur_energy == 0);
        int tgt_voiced  = (tgt_period != 0);
        int voices_differ = (now_voiced != tgt_voiced);
        int skip_interp = (!use_interp
                           || (now_silence != (tgt_energy == 0))
                           || (unvoiced_nointerp && voices_differ));

        /* Infinite-loop guard */
        if (frame_count >= max_frames
            && frame.energy_idx == 0
            && tgt_energy == 0
            && tgt_period == 0
            && !frame.repeat
            && ending_countdown == -1)
        {
            if (verbose) fprintf(stderr, "[synth] loop guard at frame %d\n", frame_count);
            break;
        }

        if (verbose)
            fprintf(stderr, "[synth] frame %3d: energy=%3d period=%3d repeat=%d silence=%d stop=%d skip_interp=%d\n",
                    frame_count, tgt_energy, tgt_period,
                    frame.repeat, frame.is_silence, frame.is_stop, skip_interp);

        /* Grow buffer if needed */
        if (nsamples + SAMPLES_PER_FRAME > cap) {
            cap *= 2;
            samples = (double *)realloc(samples, cap * sizeof(double));
        }

        int synth_energy = from_energy;
        int synth_period = from_period;
        int synth_k[10];
        for (i = 0; i < 10; i++) synth_k[i] = from_k[i];

        for (s_idx = 0; s_idx < SAMPLES_PER_FRAME; s_idx++) {
            if (!skip_interp) {
                /* Full linear interpolation: update parameters every sample.
                 * At s_idx==0 we use the previous frame's values (t=0/N),
                 * stepping toward the target so that at s_idx==N-1 we are
                 * just one step short of the target (the target is latched
                 * at the end of the frame, matching hardware behaviour). */
                int t = s_idx + 1;   /* 1 .. SAMPLES_PER_FRAME */
                synth_energy = (int)round(from_energy
                    + (tgt_energy - from_energy) * (double)t / SAMPLES_PER_FRAME);
                synth_period = (int)round(from_period
                    + (tgt_period - from_period) * (double)t / SAMPLES_PER_FRAME);
                for (i = 0; i < 10; i++)
                    synth_k[i] = (int)round(from_k[i]
                        + (tgt_k[i] - from_k[i]) * (double)t / SAMPLES_PER_FRAME);
            }
            /* When skip_interp is set, synth_* stays at from_* for the whole
             * frame (the target is latched below after the loop). */

            double u10 = excitation(&s, synth_energy, synth_period, chip->chirp, chip->chirp_len, use_pulse);
            double smp  = lattice_filter(&s, u10, synth_k);
            samples[nsamples++] = smp * OUTPUT_GAIN;
        }

        cur_energy = tgt_energy;
        cur_period = tgt_period;
        for (i = 0; i < 10; i++) cur_k[i]   = tgt_k[i];
        for (i = 0; i < 10; i++) prev_k[i]  = tgt_k[i];
        frame_count++;

        if (ending_countdown > 0) {
            ending_countdown--;
            if (ending_countdown == 0) break;
        }
    }

    *out_count = nsamples;
    return samples;
}

/* =========================================================================
 * WAV writer (16-bit mono 8 kHz)
 * ========================================================================= */
static void write_le16(FILE *f, uint16_t v)
{
    fputc(v & 0xFF, f);
    fputc((v >> 8) & 0xFF, f);
}

static void write_le32(FILE *f, uint32_t v)
{
    fputc(v & 0xFF, f);
    fputc((v >> 8) & 0xFF, f);
    fputc((v >> 16) & 0xFF, f);
    fputc((v >> 24) & 0xFF, f);
}

static int write_wav(const char *path, const double *samples, size_t n, double gain)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return -1; }

    uint32_t data_bytes = (uint32_t)(n * 2);
    uint32_t sample_rate = 8000;
    uint16_t block_align = 2;
    uint16_t bits = 16;
    uint16_t channels = 1;

    /* Find peak for normalisation (then apply user gain on top) */
    double peak = 0.0;
    size_t i;
    for (i = 0; i < n; i++) {
        double v = fabs(samples[i]);
        if (v > peak) peak = v;
    }
    double scale = (peak > 0.0) ? (gain * 32767.0 / peak) : 0.0;

    fputs("RIFF", f);
    write_le32(f, 36 + data_bytes);
    fputs("WAVE", f);
    fputs("fmt ", f);
    write_le32(f, 16);
    write_le16(f, 1);          /* PCM */
    write_le16(f, channels);
    write_le32(f, sample_rate);
    write_le32(f, sample_rate * block_align);
    write_le16(f, block_align);
    write_le16(f, bits);
    fputs("data", f);
    write_le32(f, data_bytes);

    for (i = 0; i < n; i++) {
        double v = samples[i] * scale;
        if (v >  32767.0) v =  32767.0;
        if (v < -32768.0) v = -32768.0;
        int16_t s = (int16_t)v;
        fputc(s & 0xFF, f);
        fputc((s >> 8) & 0xFF, f);
    }

    fclose(f);
    return 0;
}

/* =========================================================================
 * Argument parsing helpers
 * ========================================================================= */
static const char *arg_value(int argc, char **argv, const char *flag)
{
    int i;
    for (i = 1; i < argc - 1; i++)
        if (strcmp(argv[i], flag) == 0)
            return argv[i+1];
    return NULL;
}

static int arg_flag(int argc, char **argv, const char *flag)
{
    int i;
    for (i = 1; i < argc; i++)
        if (strcmp(argv[i], flag) == 0)
            return 1;
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s -infile INPUT -outfile OUTPUT [options]\n"
        "\n"
        "Options:\n"
        "  -infile  FILE      Input LPC binary file (required)\n"
        "  -outfile FILE      Output WAV file (required)\n"
        "  -chip    NAME      Chip variant: tms5100, tms5110, tms5200, tms5220\n"
        "                     (default: tms5220)\n"
        "  -gain    N         Audio gain 0-300 percent (default: 100)\n"
        "  -startaddr HEX     Hex byte offset into input file to start decoding\n"
        "                     (default: 0, e.g. -startaddr 1A00)\n"
        "  -maxframes N       Infinite-loop guard frame limit (default: 200)\n"
        "  -nointerp          Disable all parameter interpolation\n"
        "  -full-interp       Interpolate across voiced/unvoiced transitions\n"
        "                     (default: transitions snap like real hardware)\n"
        "  -pulse             Use a single-sample impulse instead of chirp table\n"
        "  -verbose           Print per-frame debug info\n"
        "\n"
        "Output: 16-bit mono 8 kHz WAV\n",
        prog);
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(int argc, char **argv)
{
    const char *infile    = arg_value(argc, argv, "-infile");
    const char *outfile   = arg_value(argc, argv, "-outfile");
    const char *chip_name = arg_value(argc, argv, "-chip");
    const char *gain_str  = arg_value(argc, argv, "-gain");
    const char *mf_str    = arg_value(argc, argv, "-maxframes");
    const char *addr_str  = arg_value(argc, argv, "-startaddr");
    int use_interp        = !arg_flag(argc, argv, "-nointerp");
    int unvoiced_nointerp = !arg_flag(argc, argv, "-full-interp"); /* on by default */
    int use_pulse         = arg_flag(argc, argv, "-pulse");
    int verbose           = arg_flag(argc, argv, "-verbose");

    if (!infile || !outfile) {
        usage(argv[0]);
        return 1;
    }

    if (!chip_name) chip_name = "tms5220";
    double gain       = gain_str ? atof(gain_str) / 100.0 : 1.00;
    int    max_frames = mf_str   ? atoi(mf_str)            : 200;
    long   start_addr = addr_str ? strtol(addr_str, NULL, 16) : 0;

    const ChipDef *chip = find_chip(chip_name);
    if (!chip) {
        fprintf(stderr, "Unknown chip '%s'. Choose: tms5100, tms5110, tms5200, tms5220\n", chip_name);
        return 1;
    }

    /* Read input file */
    FILE *fin = fopen(infile, "rb");
    if (!fin) { perror(infile); return 1; }
    fseek(fin, 0, SEEK_END);
    long fsize = ftell(fin);
    rewind(fin);
    if (fsize <= 0) { fprintf(stderr, "Empty input file\n"); fclose(fin); return 1; }

    if (start_addr < 0 || start_addr >= fsize) {
        fprintf(stderr, "Start address 0x%lx is out of range (file is %ld bytes)\n",
                start_addr, fsize);
        fclose(fin);
        return 1;
    }

    long data_len = fsize - start_addr;
    uint8_t *data = (uint8_t *)malloc((size_t)data_len);
    if (!data) { fprintf(stderr, "Out of memory\n"); fclose(fin); return 1; }
    if (fseek(fin, start_addr, SEEK_SET) != 0) {
        fprintf(stderr, "Seek error\n"); free(data); fclose(fin); return 1;
    }
    if ((long)fread(data, 1, (size_t)data_len, fin) != data_len) {
        fprintf(stderr, "Read error\n"); free(data); fclose(fin); return 1;
    }
    fclose(fin);

    fprintf(stderr, "Chip: %s | Input: %ld bytes", chip->name, data_len);
    if (start_addr) fprintf(stderr, " (offset 0x%lx)", start_addr);
    fprintf(stderr, " | Gain: %.0f%%\n", gain * 100.0);

    /* Synthesize */
    size_t  nsamples = 0;
    double *samples  = synthesize(data, (size_t)data_len, chip,
                                  use_interp, unvoiced_nointerp, use_pulse,
                                  max_frames, verbose, &nsamples);
    free(data);

    if (!samples || nsamples == 0) {
        fprintf(stderr, "No samples produced\n");
        free(samples);
        return 1;
    }

    fprintf(stderr, "Synthesized %zu samples (%.2f seconds)\n",
            nsamples, (double)nsamples / 8000.0);

    /* Write WAV */
    if (write_wav(outfile, samples, nsamples, gain) != 0) {
        free(samples);
        return 1;
    }
    free(samples);

    fprintf(stderr, "Written: %s\n", outfile);
    return 0;
}
