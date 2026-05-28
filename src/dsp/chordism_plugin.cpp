/*
 * Chordism DSP Plugin for Schwung (Move Anything)
 *
 * MIT License. See LICENSE.
 *
 * Milestone 3a: 4-voice sine chord at fixed intervals
 * (root, M3, P5, octave). Mono last-note priority with
 * clean voice-steal release.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

extern "C" {

#define MOVE_PLUGIN_API_VERSION_2 2

typedef struct host_api_v1 {
    uint32_t api_version;
    int sample_rate;
    int frames_per_block;
    uint8_t *mapped_memory;
    int audio_out_offset;
    int audio_in_offset;
    void (*log)(const char *msg);
    int (*midi_send_internal)(const uint8_t *msg, int len);
    int (*midi_send_external)(const uint8_t *msg, int len);
} host_api_v1_t;

typedef struct plugin_api_v2 {
    uint32_t api_version;
    void* (*create_instance)(const char *module_dir, const char *json_defaults);
    void (*destroy_instance)(void *instance);
    void (*on_midi)(void *instance, const uint8_t *msg, int len, int source);
    void (*set_param)(void *instance, const char *key, const char *val);
    int (*get_param)(void *instance, const char *key, char *buf, int buf_len);
    int (*get_error)(void *instance, char *buf, int buf_len);
    void (*render_block)(void *instance, int16_t *out_interleaved_lr, int frames);
} plugin_api_v2_t;

}

static const host_api_v1_t *g_host = nullptr;

static const float SAMPLE_RATE = 44100.0f;
static const float TWO_PI = 6.28318530717958647692f;

static const int   CHORD_SIZE = 4;
static const int   NUM_VOICES = 16;  /* 4 banks of CHORD_SIZE — release tails finish before steal */
static const int   HELD_STACK_MAX = 16;

/* Chord LUT — DVNA-spec chord types. Each row is CHORD_SIZE semitone offsets
 * from the root note (in MIDI semitones, so 12 = octave). */
enum ChordType {
    CHORD_UNISON_OCTAVES = 0,
    CHORD_FIFTH,
    CHORD_MINOR,
    CHORD_MINOR_7,
    CHORD_MINOR_9,
    CHORD_MINOR_11,
    CHORD_MAJOR,
    CHORD_MAJOR_7,
    CHORD_MAJOR_9,
    CHORD_SUS_4,
    CHORD_SIX_NINE,
    CHORD_MINOR_6,
    CHORD_TENTH,
    CHORD_DOMINANT_7,
    CHORD_DOMINANT_7_B9,
    CHORD_HALF_DIMINISHED,
    NUM_CHORDS
};

static const int CHORD_TABLE[NUM_CHORDS][CHORD_SIZE] = {
    { 0, 12, 24, 36 },   /* Unison/Octaves */
    { 0,  7, 12, 19 },   /* Fifth */
    { 0,  3,  7, 12 },   /* Minor */
    { 0,  3,  7, 10 },   /* Minor 7 */
    { 0,  3,  7, 14 },   /* Minor 9 */
    { 0,  3,  7, 17 },   /* Minor 11 */
    { 0,  4,  7, 12 },   /* Major */
    { 0,  4,  7, 11 },   /* Major 7 */
    { 0,  4,  7, 14 },   /* Major 9 */
    { 0,  5,  7, 12 },   /* Suspended 4 */
    { 0,  4,  9, 14 },   /* 6/9 */
    { 0,  3,  7,  9 },   /* Minor 6 */
    { 0,  4,  7, 16 },   /* 10th */
    { 0,  4,  7, 10 },   /* Dominant 7 */
    { 0,  4,  7, 13 },   /* Dominant 7 / b9 */
    { 0,  3,  6, 10 },   /* Half Diminished */
};

/* Attack range: 1 ms .. 4 s, linear ramp.
 * Release range: 5 ms .. 4 s, exponential decay (asymptotic). */
static const float ATTACK_MIN_S = 0.001f;
static const float ATTACK_MAX_S = 4.0f;
static const float RELEASE_MIN_S = 0.005f;
static const float RELEASE_MAX_S = 4.0f;

/* Silence threshold below which the envelope clamps to 0 and the voice idles. */
static const float ENV_SILENCE = 1e-4f;

/* Detune max — applied as (chord_step_index * detune * MAX_DETUNE_CENTS).
 * 50 cents per step means voice 3 can be up to 150 cents (1.5 semitones)
 * sharp of voice 0 at full detune. */
static const float MAX_DETUNE_CENTS = 50.0f;

/* Pan morph LUT — 16 hand-authored 4-voice pan rows. Each value is -1
 * (hard left) to +1 (hard right). Added on top of width-based spread. */
static const float PAN_MORPH_LUT[16][CHORD_SIZE] = {
    {  0.0f,  0.0f,  0.0f,  0.0f },  /* 0  all center */
    { -1.0f, -0.3f,  0.3f,  1.0f },  /* 1  L→R ramp */
    {  1.0f,  0.3f, -0.3f, -1.0f },  /* 2  R→L ramp */
    { -1.0f,  1.0f, -1.0f,  1.0f },  /* 3  alt LRLR */
    {  1.0f, -1.0f,  1.0f, -1.0f },  /* 4  alt RLRL */
    { -1.0f, -1.0f,  1.0f,  1.0f },  /* 5  LL/RR split */
    { -1.0f,  0.0f,  0.0f,  1.0f },  /* 6  outer extremes */
    {  0.0f, -1.0f,  1.0f,  0.0f },  /* 7  inner extremes */
    { -0.5f,  0.5f, -0.5f,  0.5f },  /* 8  zigzag */
    {  0.5f, -0.5f,  0.5f, -0.5f },  /* 9  inv zigzag */
    { -1.0f, -0.5f,  0.5f,  1.0f },  /* 10 wide ramp */
    {  0.0f, -1.0f,  0.0f,  1.0f },  /* 11 L kick / R kick */
    {  0.0f,  1.0f,  0.0f, -1.0f },  /* 12 R kick / L kick */
    { -1.0f,  1.0f,  0.0f,  0.0f },  /* 13 first two outer */
    {  0.0f,  0.0f,  1.0f, -1.0f },  /* 14 last two outer */
    {  0.7f, -0.7f, -0.7f,  0.7f },  /* 15 outside-in */
};

/* Level morph LUT — 16 hand-authored 4-voice gain rows, musically useful
 * variations of the chord mix. morph_index linearly interpolates between
 * adjacent rows; morph_intensity blends the row toward "all 1.0" (flat). */
static const int NUM_LEVEL_MORPHS = 16;
static const float LEVEL_MORPH_LUT[NUM_LEVEL_MORPHS][CHORD_SIZE] = {
    {1.0f, 1.0f, 1.0f, 1.0f},  /* 0  all equal */
    {0.2f, 0.5f, 0.8f, 1.0f},  /* 1  ramp up */
    {1.0f, 0.8f, 0.5f, 0.2f},  /* 2  ramp down */
    {1.0f, 0.0f, 0.0f, 0.0f},  /* 3  root only */
    {0.0f, 0.0f, 0.0f, 1.0f},  /* 4  top only */
    {1.0f, 0.0f, 0.0f, 1.0f},  /* 5  root + top */
    {0.0f, 1.0f, 1.0f, 0.0f},  /* 6  inner pair */
    {1.0f, 0.0f, 1.0f, 0.0f},  /* 7  odd */
    {0.0f, 1.0f, 0.0f, 1.0f},  /* 8  even */
    {0.1f, 0.3f, 0.7f, 1.0f},  /* 9  log up */
    {0.3f, 1.0f, 1.0f, 0.3f},  /* 10 middle-peak triangle */
    {1.0f, 0.3f, 0.3f, 1.0f},  /* 11 inv-triangle */
    {0.0f, 0.0f, 0.5f, 1.0f},  /* 12 top-heavy */
    {1.0f, 0.7f, 0.3f, 0.0f},  /* 13 bottom-heavy */
    {1.0f, 1.0f, 0.5f, 0.5f},  /* 14 low pair */
    {0.5f, 0.5f, 1.0f, 1.0f},  /* 15 high pair */
};

/* Reverb (Schroeder-style: 4 parallel combs + 2 series allpass per channel).
 * Comb delays staggered with prime-ish lengths to avoid resonant pile-up.
 * Allpass delays much shorter than combs (diffusion stage). Stereo offset
 * applied to right-channel delay lengths for natural width. */
/* Delay — 65536-sample ring buffer (~1.486 s @ 44.1k). Power-of-two for cheap
 * mask-based wrap. Per-channel (true stereo delay). */
static const int DELAY_BUFFER_SIZE = 65536;
static const int DELAY_BUFFER_MASK = DELAY_BUFFER_SIZE - 1;
static const float DELAY_TIME_MIN_S = 0.005f;
static const float DELAY_TIME_MAX_S = 1.4f;
static const float DELAY_FEEDBACK_MAX = 0.95f;

enum DelayMode { DELAY_STEREO = 0, DELAY_PINGPONG = 1, DELAY_FLIPFLOP = 2 };
static const int NUM_DELAY_MODES = 3;

static const int REVERB_COMB_L[4] = { 1557, 1617, 1491, 1422 };
static const int REVERB_COMB_R[4] = { 1580, 1640, 1514, 1445 };
static const int REVERB_ALLPASS_L[2] = { 556, 441 };
static const int REVERB_ALLPASS_R[2] = { 579, 464 };
static const int REVERB_COMB_MAX = 1700;
static const int REVERB_ALLPASS_MAX = 600;

struct CombFilter {
    float buffer[REVERB_COMB_MAX];
    int size;        /* current delay length */
    int idx;
    float filter;    /* damping low-pass state */
    float feedback;
    float damp;
};

struct AllpassFilter {
    float buffer[REVERB_ALLPASS_MAX];
    int size;
    int idx;
};

static inline float comb_process(CombFilter *c, float input) {
    float out = c->buffer[c->idx];
    c->filter = out * (1.0f - c->damp) + c->filter * c->damp;
    c->buffer[c->idx] = input + c->filter * c->feedback;
    c->idx++;
    if (c->idx >= c->size) c->idx = 0;
    return out;
}

static inline float allpass_process(AllpassFilter *a, float input) {
    float bufout = a->buffer[a->idx];
    float out = -input + bufout;
    a->buffer[a->idx] = input + bufout * 0.5f;
    a->idx++;
    if (a->idx >= a->size) a->idx = 0;
    return out;
}

/* Vibrato range. Speed exp-mapped 0.1..12 Hz, depth 0..100 cents, delay
 * linear 0..2 sec rise time (ramp from 0 to full depth after note-on). */
static const float VIB_SPEED_MIN_HZ = 0.1f;
static const float VIB_SPEED_MAX_HZ = 12.0f;
static const float VIB_DEPTH_MAX_CENTS = 100.0f;
static const float VIB_DELAY_MAX_S = 2.0f;
static const float ONE_OVER_1200 = 1.0f / 1200.0f;
static const float ONE_OVER_12 = 1.0f / 12.0f;

/* Pitch sweep: ±12 semitones max offset at amount=±1. Rate sets exp decay
 * time toward 0. */
static const float SWEEP_MAX_SEMITONES = 12.0f;
static const float SWEEP_TIME_MIN_S = 0.01f;
static const float SWEEP_TIME_MAX_S = 4.0f;

/* Glide: portamento time at rate=1.0 (rate=0 → snap). */
static const float GLIDE_TIME_MAX_S = 2.0f;

enum ArpDirection { ARP_UP = 0, ARP_DOWN, ARP_UPDOWN, ARP_RANDOM };
static const int NUM_ARP_DIRECTIONS = 4;
static const float ARP_BPM_MIN = 30.0f;
static const float ARP_BPM_MAX = 240.0f;

/* Preset — snapshot of meaningful parameters for instant recall. */
struct Preset {
    const char *name;
    int chord_type;
    float chord_spread;
    float chord_rotation;
    float detune;
    float width;
    int waveforms[CHORD_SIZE];
    float shapes[CHORD_SIZE];
    float morph_index;
    float morph_intensity;
    float pan_morph_index;
    float pan_morph_intensity;
    int fm_modulator_idx;
    float fm_amount;
    float lfo_rate;
    float lfo_depth;
    int lfo_shape;
    float vib_depth;
    float vib_speed;
    float vib_delay;
    float sweep_amount;
    float sweep_rate;
    float glide_rate;
    float filter_cutoff;
    float filter_resonance;
    int filter_mode;
    int filter_slope;
    float filter_env_attack;
    float filter_env_decay;
    float filter_env_depth;
    float filter_lfo_rate;
    float filter_lfo_depth;
    float filter_lfo_spread;
    int filter_lfo_shape;
    float drive;
    float attack;
    float release;
    float volume;
    float reverb_mix;
    float reverb_decay;
    float reverb_damp;
    float delay_mix;
    float delay_time;
    float delay_feedback;
    float delay_tone;
    float grind;
    float bit_shift;
    float decimator;
    int arp_enabled;
    float arp_tempo;
    int arp_direction;
    int delay_mode;
    int vib_stray;
    int glide_legato;
};

static const Preset PRESETS[] = {
/*  name           chord  spread rot detune width  waveforms          shapes              morphI morphIn panI panIn fmM fmA  lfoR lfoD lfoS vibD vibS vibDly swpA swpR gld  cutoff reso mode slope fEnvA fEnvD fEnvAmt fLfoR fLfoD fLfoS fLfoSh drive  A     R     vol   rvMix rvDec rvDmp dlMix dlT  dlFb dlTn grnd bShf dec   arp tempo dir dlyMd vStray glLeg
*/
{"Init",            6,    0.5f,  0.0f, 0.0f,  1.0f,  {1,1,1,1},        {0,0,0,0},          0,    0,      0,   0,    0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,1.0f,  0.0f,0,   0,    0.0f, 0.3f, 0.0f,    0.0f, 0.0f, 0.0f, 0,    0.0f,  0.05f,0.30f,0.8f, 0.0f, 0.5f, 0.3f, 0.0f, 0.3f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Lush Pad",        8,    0.5f,  0.0f, 0.30f, 1.0f,  {3,3,3,3},        {0.4f,0.4f,0.4f,0.4f}, 0.4f, 0.5f,   0.5f, 0.6f, 0,  0.0f,0.15f,0.4f,0,  0.3f,0.3f,0.3f,  0.0f,0.5f,0.4f,0.5f,  0.4f,0,   0,    0.1f, 0.6f, 0.3f,    0.05f,0.3f, 0.3f, 0,    0.05f, 0.25f,0.60f,0.7f, 0.55f,0.7f, 0.4f, 0.1f, 0.35f,0.3f,0.5f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"FM Bell",         6,    0.5f,  0.0f, 0.05f, 0.8f,  {1,1,1,1},        {0,0,0,0},          0.2f, 0.3f,   0.0f, 0.0f, 0,  0.40f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.7f,  0.0f,0,   0,    0.0f, 0.5f, 0.0f,    0.0f, 0.0f, 0.0f, 0,    0.0f,  0.0f, 0.55f,0.7f, 0.35f,0.6f, 0.2f, 0.0f, 0.3f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Plucky Lead",     13,   0.5f,  0.0f, 0.0f,  0.6f,  {3,3,3,3},        {0.2f,0.2f,0.2f,0.2f}, 0,   0,      0,   0,    0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.45f, 0.0f,0,   0,    0.0f, 0.25f, 0.85f,  0.0f, 0.0f, 0.0f, 0,    0.0f,  0.0f, 0.30f,0.7f, 0.2f, 0.5f, 0.4f, 0.15f,0.4f,0.45f,0.5f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Filtered Sweep",  6,    0.5f,  0.0f, 0.0f,  0.8f,  {3,3,3,3},        {0,0,0,0},          0,   0,      0.4f, 0.4f, 0,  0.0f,0.45f,0.4f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.45f, 0.4f, 0,   0,    0.0f, 0.5f, 0.0f,    0.4f, 0.35f,0.3f, 0,    0.10f, 0.05f,0.40f,0.6f, 0.4f, 0.7f, 0.3f, 0.0f, 0.3f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Ambient Drone",   0,    0.5f,  0.0f, 0.10f, 0.5f,  {1,2,2,1},        {0.6f,0.6f,0.3f,0.3f}, 0.3f, 0.25f,  0.0f, 0.0f, 0,  0.0f,0.10f,0.3f,0, 0.2f,0.45f,0.6f,  0.0f,0.5f,0.0f,0.5f,  0.5f,0,   0,    0.0f, 0.5f, 0.0f,    0.10f,0.4f, 0.5f, 0,    0.0f,  0.5f, 0.80f,0.7f, 0.7f, 0.85f,0.3f, 0.3f, 0.6f,0.55f,0.5f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Arp Bliss",       6,    0.5f,  0.0f, 0.0f,  0.9f,  {3,3,3,3},        {0,0,0,0},          0,   0,      0.3f, 0.4f, 0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.5f,  0.0f,0,   0,    0.0f, 0.5f, 0.0f,    0.0f, 0.0f, 0.0f, 0,    0.0f,  0.0f, 0.30f,0.7f, 0.4f, 0.6f, 0.4f, 0.3f, 0.4f,0.55f,0.6f,0.0f,0.0f,0.0f,  1,  0.75f, 0,  0,    0,    0},
{"Lo-Fi Stab",      2,    0.5f,  0.0f, 0.0f,  0.7f,  {3,3,3,3},        {0,0,0,0},          0,   0,      0.0f, 0.0f, 0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.40f, 0.0f,0,   0,    0.0f, 0.35f, 0.4f,   0.0f, 0.0f, 0.0f, 0,    0.0f,  0.0f, 0.25f,0.7f, 0.2f, 0.5f, 0.3f, 0.0f, 0.3f,0.4f,0.7f,0.5f,0.3f,0.5f,  0,  0.4f,  0,  0,    0,    0},

/* === New in 0.2.2 === */
{"Glacial",         8,    0.5f,  0.0f, 0.20f, 0.6f,  {3,3,3,3},        {0.3f,0.3f,0.3f,0.3f}, 0.3f, 0.4f,   0.2f, 0.3f, 0,  0.0f,0.15f,0.3f,0,  0.1f,0.3f,0.6f,  0.0f,0.5f,0.0f,0.45f, 0.2f, 0,   0,    0.5f, 0.7f, 0.10f,   0.12f,0.25f,0.7f, 0,    0.05f, 0.70f,0.85f,0.6f, 0.75f,0.9f, 0.4f, 0.2f, 0.55f,0.5f,0.55f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Acid Lead",       1,    0.4f,  0.0f, 0.0f,  0.4f,  {3,3,0,0},        {0.2f,0.2f,0,0},    0,   0,      0,   0,    0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.25f, 0.75f, 0,   1,    0.0f, 0.30f, 0.55f,  0.0f, 0.0f, 0.0f, 0,    0.35f, 0.0f, 0.20f,0.6f, 0.20f,0.5f, 0.3f, 0.20f,0.4f,0.55f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    1},
{"Sub Bass",        0,    0.4f,  0.0f, 0.0f,  0.15f, {1,0,0,0},        {0,0,0,0},          0,   0,      0,   0,    0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.10f,0.65f, 0.0f, 0,   0,    0.0f, 0.30f, 0.0f,    0.0f, 0.0f, 0.0f, 0,    0.0f,  0.0f, 0.30f,0.85f,0.05f,0.4f, 0.3f, 0.0f, 0.3f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"EP",              7,    0.5f,  0.0f, 0.05f, 0.5f,  {1,1,1,1},        {0.4f,0.3f,0.3f,0.4f}, 0.2f, 0.3f,   0.0f, 0.0f, 0,  0.15f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.70f, 0.1f, 0,   0,    0.0f, 0.50f, -0.30f, 0.0f, 0.0f, 0.0f, 0,    0.0f,  0.0f, 0.45f,0.7f, 0.40f,0.6f, 0.4f, 0.1f, 0.4f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Synth Brass",     6,    0.5f,  0.0f, 0.10f, 0.7f,  {3,3,3,3},        {0.1f,0.1f,0.1f,0.1f}, 0,   0,      0,   0,    0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.40f, 0.15f, 0,   0,    0.05f,0.50f, 0.50f,   0.0f, 0.0f, 0.0f, 0,    0.15f, 0.05f,0.35f,0.7f, 0.35f,0.55f,0.3f, 0.0f, 0.3f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Reese Bass",      0,    0.4f,  0.0f, 0.70f, 0.5f,  {3,3,3,3},        {0.15f,0.15f,0.15f,0.15f}, 0, 0,    0,   0,    0,  0.0f,0.1f,0.15f,0, 0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.45f, 0.3f, 0,   1,    0.0f, 0.50f, 0.0f,    0.0f, 0.0f, 0.0f, 0,    0.20f, 0.0f, 0.40f,0.65f,0.2f, 0.5f, 0.4f, 0.0f, 0.3f,0.4f,0.7f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  0,    0,    0},
{"Underwater",      8,    0.6f,  0.0f, 0.15f, 0.7f,  {1,2,2,1},        {0.5f,0.5f,0.5f,0.5f}, 0.5f, 0.5f,   0.4f, 0.7f, 0,  0.0f,0.20f,0.4f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.10f,0.50f, 0.4f, 0,   0,    0.30f, 0.7f, 0.20f,  0.15f,0.4f, 0.9f, 0,    0.0f,  0.50f,0.70f,0.6f, 0.55f,0.7f, 0.4f, 0.35f,0.5f,0.6f,0.5f,0.0f,0.0f,0.0f,  0,  0.4f,  0,  2,    0,    0},
{"Trance Arp",      6,    0.5f,  0.0f, 0.05f, 0.8f,  {3,3,3,3},        {0.2f,0.2f,0.2f,0.2f}, 0,   0,      0,   0,    0,  0.0f,0.0f,0.0f,0,  0.0f,0.5f,0.2f,  0.0f,0.5f,0.0f,0.45f, 0.5f, 0,   1,    0.0f, 0.25f, 0.55f,   0.0f, 0.0f, 0.0f, 0,    0.20f, 0.0f, 0.15f,0.6f, 0.30f,0.5f, 0.4f, 0.35f,0.3f,0.55f,0.7f,0.0f,0.0f,0.0f,  1,  0.85f, 2,  1,    0,    0},
};
static const int NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

/* Filter cutoff range, exp-mapped 0..1 → 20 Hz .. 20 kHz. TPT SVF is stable
 * up to Nyquist but tan() blows up exactly AT Nyquist — keep cutoff under
 * 0.49 * SR. */
static const float FILTER_CUTOFF_MIN_HZ = 20.0f;
static const float FILTER_CUTOFF_MAX_HZ = 20000.0f;
/* Resonance maps to Q (peak height). 0 = broad (Q=0.5, Butterworth-ish),
 * 1 = very resonant (Q=20). */
static const float FILTER_Q_MIN = 0.5f;
static const float FILTER_Q_MAX = 20.0f;

/* TPT (Topology-Preserving Transform) SVF state — Vadim Zavalishin /
 * Andy Simper formulation. Stable for any g and k. */
struct SVF {
    float ic1eq;
    float ic2eq;
};

enum FilterMode { FILT_LP = 0, FILT_HP = 1, FILT_BP = 2 };
static const int NUM_FILTER_MODES = 3;

enum EnvStage { ENV_IDLE, ENV_ATTACK, ENV_HOLD, ENV_RELEASE };

enum Waveform { WAVE_OFF = 0, WAVE_SINE = 1, WAVE_TRIANGLE = 2, WAVE_SAW = 3, WAVE_SQUARE = 4 };
static const int NUM_WAVEFORMS = 5;

enum LFOShape { LFO_TRIANGLE = 0, LFO_RAMP_UP = 1, LFO_RAMP_DOWN = 2, LFO_SQUARE = 3 };
static const int NUM_LFO_SHAPES = 4;

/* LFO rate range, exp-mapped: 0.01 Hz (very slow swell) to 10 Hz (vibrato-ish). */
static const float LFO_RATE_MIN_HZ = 0.01f;
static const float LFO_RATE_MAX_HZ = 10.0f;

struct LFO {
    float phase;      /* 0..1 */
    float phase_inc;  /* per audio sample */
    int   shape;      /* 0..NUM_LFO_SHAPES-1 */
};

struct AREnv {
    EnvStage stage;
    float value;
    float attack_inc;
    float release_coef;
};

/* AD (Attack-Decay) envelope — note-triggered, ignores gate. Shares attack
 * timing with AR's attack_inc and decay timing with release_coef. */
struct ADEnv {
    EnvStage stage;
    float value;
    float attack_inc;
    float decay_coef;
};

struct Voice {
    bool active;
    int root_note;       /* MIDI root that spawned this voice (for note-off match) */
    int chord_step;      /* 0..CHORD_SIZE-1 — position in the chord, for morph lookups */
    int waveform;        /* per-voice waveform — set at voice_start */
    float phase;
    float phase_inc;
    float target_phase_inc;  /* glide target — phase_inc ramps toward this */
    float glide_step;        /* per-sample addition; 0 = no glide */
    float velocity;
    float pan_offset;    /* (chord_step_norm - 0.5), -0.5..+0.5. Pan scales by inst->width per sample. */
    AREnv env;
};

struct chordism_instance_t {
    char module_dir[512];

    /* Params */
    float attack;        /* 0..1 */
    float release;       /* 0..1 */
    float volume;        /* 0..1 */
    int   waveforms[CHORD_SIZE];  /* per-osc waveform (chord step → wave) */
    float shapes[CHORD_SIZE];     /* per-osc shape */
    float morph_index;   /* 0..1 — sweeps level-morph LUT */
    float morph_intensity; /* 0..1 — blend flat→LUT row */
    float morph_gains[CHORD_SIZE];  /* cached effective gain per chord step */

    float pan_morph_index;     /* 0..1 — sweeps pan-morph LUT */
    float pan_morph_intensity; /* 0..1 — blend center→LUT row */
    float pan_morph_pans[CHORD_SIZE]; /* cached effective pan per chord step (-1..+1) */

    /* Chord voicing: spread scales intervals, rotation shifts which osc plays
     * which chord degree. */
    float chord_spread;     /* 0..1 → interval multiplier 0..2 */
    float chord_rotation;   /* 0..1 → integer rotation 0..CHORD_SIZE-1 */

    /* FM: phase modulation. fm_modulator_idx picks which chord_step is the
     * modulator; carriers (other steps) get phase += last_modulator * fm_amount. */
    int   fm_modulator_idx;   /* 0..CHORD_SIZE-1 */
    float fm_amount;          /* 0..1 → 0..2π phase modulation depth */
    float last_modulator_sample;
    float lfo_rate;      /* 0..1 — exp-mapped to Hz */
    float lfo_depth;     /* 0..1 — modulation amount on shape */
    int   lfo_shape;     /* 0..NUM_LFO_SHAPES-1 */
    int   chord_type;    /* 0..NUM_CHORDS-1 — row in CHORD_TABLE */
    float detune;        /* 0..1 → 0..MAX_DETUNE_CENTS per chord step */
    float width;         /* 0..1 — 0=mono, 1=full LR spread across chord */
    float vib_speed;     /* 0..1 → exp 0.1..12 Hz */
    float vib_depth;     /* 0..1 → 0..VIB_DEPTH_MAX_CENTS */
    float vib_delay;     /* 0..1 → 0..VIB_DELAY_MAX_S rise time */
    float sweep_amount;  /* -1..+1 → ±12 semitones offset at note-on */
    float sweep_rate;    /* 0..1 → exp time SWEEP_TIME_MIN_S..MAX_S */

    /* Vibrato runtime state — single global LFO + ramp shared by all voices. */
    float vibrato_phase;
    float vibrato_phase_inc;
    float vib_ramp_value;   /* 0..1, climbs after chord_on */
    float vib_ramp_inc;     /* per-sample ramp step (precomputed from vib_delay) */

    /* Pitch sweep runtime state — single offset, decays toward 0. */
    float sweep_value;        /* current semitones */
    float sweep_coef;         /* exp decay coef per sample (precomputed) */
    float filter_cutoff;     /* 0..1 → exp 20..20kHz */
    float filter_resonance;  /* 0..1 → Q from FILTER_Q_MIN to FILTER_Q_MAX */
    int   filter_mode;       /* 0..NUM_FILTER_MODES-1 */
    float filter_env_attack; /* 0..1 → linear A time */
    float filter_env_decay;  /* 0..1 → exp D time */
    float filter_env_depth;  /* -1..+1 — bipolar modulation of cutoff */
    float drive;             /* 0..1 → 1..10x pre-tanh gain */

    SVF   filter_l;
    SVF   filter_r;
    /* Second SVF stage per channel — used when filter_slope = 24 dB (cascade).
     * For 12 dB mode the b-stages are inert. */
    SVF   filter_l_b;
    SVF   filter_r_b;
    /* Static (no-modulation) TPT SVF coefficients. Per-sample recompute kicks
     * in when filter_env_depth != 0 OR filter_lfo_depth != 0. */
    float filter_a1;
    float filter_a2;
    float filter_a3;
    float filter_k;
    int   filter_slope;  /* 0 = 12 dB, 1 = 24 dB */

    /* Filter LFO — dedicated, separate from shape LFO. Stereo spread offsets
     * the R-channel LFO phase from L for auto-pan / stereo movement. */
    float filter_lfo_rate;
    float filter_lfo_depth;
    float filter_lfo_spread;     /* 0..1 → 0..0.5 phase offset between L and R */
    int   filter_lfo_shape;
    float filter_lfo_phase;
    float filter_lfo_phase_inc;

    ADEnv filter_env;

    /* Lo-Fi (Quality Control) */
    float grind;          /* 0..1 → bit reduction */
    float bit_shift;      /* 0..1 → DC offset bias pre-crush */
    float decimator;      /* 0..1 → sample-rate hold (0=off, 1=hold 32 samples) */
    int   decim_counter;
    float decim_hold_l;
    float decim_hold_r;

    /* Reverb */
    float reverb_mix;    /* 0..1 dry/wet */
    float reverb_decay;  /* 0..1 → comb feedback */
    float reverb_damp;   /* 0..1 → HF damping in feedback */
    CombFilter rev_comb_l[4];
    CombFilter rev_comb_r[4];
    AllpassFilter rev_ap_l[2];
    AllpassFilter rev_ap_r[2];

    /* Delay */
    float delay_mix;
    float delay_time;       /* 0..1 → DELAY_TIME_MIN_S..MAX_S */
    float delay_feedback;   /* 0..1 → 0..DELAY_FEEDBACK_MAX */
    float delay_tone;       /* 0..1 → bright LP coef on feedback */
    int   delay_mode;       /* 0=stereo, 1=ping-pong, 2=flip-flop */
    float delay_buf_l[DELAY_BUFFER_SIZE];
    float delay_buf_r[DELAY_BUFFER_SIZE];
    int   delay_write_idx;
    float delay_lp_l;
    float delay_lp_r;
    int   delay_flip_counter;  /* increments per delay-period; even=L→R, odd=R→L */

    LFO   shape_lfo;
    Voice voices[NUM_VOICES];
    int next_chord_base;   /* 0, 4, 8, 12 — round-robin between voice banks */

    /* Held-note stack — mono priority with note-off return-to-previously-held. */
    int held_notes[HELD_STACK_MAX];
    int held_count;

    /* Aftertouch (channel pressure). Routed to vibrato depth boost. */
    float aftertouch;

    int   preset_index;     /* 0..NUM_PRESETS-1 */

    /* Glide */
    float glide_rate;                       /* 0..1 → 0..GLIDE_TIME_MAX_S */
    int   glide_legato;                     /* 0=always, 1=only when overlapping notes */
    bool  has_prev_chord;                   /* false until first chord_on completes */
    bool  prev_chord_still_held;            /* true if previous note hasn't been released yet (legato) */
    float prev_phase_inc[CHORD_SIZE];       /* last chord's per-step base phase_inc */

    /* Vibrato extras */
    int   vib_stray;                        /* 0=periodic LFO, 1=smoothed-random */
    float vib_random_value;
    float vib_random_target;
    int   vib_random_counter;

    /* Arpeggiator */
    int   arp_enabled;                      /* 0/1 */
    float arp_tempo;                        /* 0..1 → 30..240 BPM */
    int   arp_direction;                    /* 0=up, 1=down, 2=updown, 3=random */
    int   arp_step_idx;
    int   arp_step_dir;                     /* +1 or -1 (for updown) */
    int   arp_sample_counter;
    int   arp_step_period;                  /* samples per arp step */
};

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

static float midi_to_hz(int note) {
    return 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
}

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static float param_from_string(const char *val, float fallback) {
    if (!val) return fallback;
    char *end = nullptr;
    float v = strtof(val, &end);
    if (end == val) return fallback;
    return clampf(v, 0.0f, 1.0f);
}

static float lerp01(float t, float lo, float hi) {
    return lo + (hi - lo) * t;
}

/* PolyBLEP — Polynomial Bandlimited stEP correction. Subtracts the
 * high-frequency content introduced by a discontinuous waveform transition
 * at sample t (phase in [0,1)). dt is phase_inc per sample. Returns 0
 * outside the small neighborhood around the discontinuity. */
static float poly_blep(float t, float dt) {
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1.0f;
    } else if (t > 1.0f - dt) {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

static float lfo_sample(int shape, float phase) {
    switch (shape) {
        case LFO_TRIANGLE:  return 4.0f * fabsf(phase - 0.5f) - 1.0f;
        case LFO_RAMP_UP:   return 2.0f * phase - 1.0f;
        case LFO_RAMP_DOWN: return 1.0f - 2.0f * phase;
        case LFO_SQUARE:    return (phase < 0.5f) ? 1.0f : -1.0f;
        default:            return 0.0f;
    }
}

/* Exp-map [0,1] to [LFO_RATE_MIN_HZ, LFO_RATE_MAX_HZ]. */
static float lfo_rate_to_hz(float rate01) {
    float ratio = LFO_RATE_MAX_HZ / LFO_RATE_MIN_HZ;
    return LFO_RATE_MIN_HZ * powf(ratio, rate01);
}

static float vib_speed_to_hz(float speed01) {
    float ratio = VIB_SPEED_MAX_HZ / VIB_SPEED_MIN_HZ;
    return VIB_SPEED_MIN_HZ * powf(ratio, speed01);
}

static void pan_morph_recompute(chordism_instance_t *inst) {
    const int N = 16;
    float idx = inst->pan_morph_index * (float)(N - 1);
    int lo = (int)idx;
    int hi = lo + 1;
    if (lo < 0) lo = 0;
    if (lo >= N) lo = N - 1;
    if (hi >= N) hi = N - 1;
    float frac = idx - (float)lo;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    for (int i = 0; i < CHORD_SIZE; ++i) {
        float lut = PAN_MORPH_LUT[lo][i] * (1.0f - frac)
                  + PAN_MORPH_LUT[hi][i] * frac;
        inst->pan_morph_pans[i] = lut * inst->pan_morph_intensity;
    }
}

static void morph_recompute(chordism_instance_t *inst) {
    float idx = inst->morph_index * (float)(NUM_LEVEL_MORPHS - 1);
    int lo = (int)idx;
    int hi = lo + 1;
    if (lo < 0) lo = 0;
    if (lo >= NUM_LEVEL_MORPHS) lo = NUM_LEVEL_MORPHS - 1;
    if (hi >= NUM_LEVEL_MORPHS) hi = NUM_LEVEL_MORPHS - 1;
    float frac = idx - (float)lo;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    for (int i = 0; i < CHORD_SIZE; ++i) {
        float lut = LEVEL_MORPH_LUT[lo][i] * (1.0f - frac)
                  + LEVEL_MORPH_LUT[hi][i] * frac;
        /* Blend flat (1.0) → lut row by intensity. */
        inst->morph_gains[i] = 1.0f + (lut - 1.0f) * inst->morph_intensity;
    }
}

static void reverb_init(chordism_instance_t *inst) {
    for (int i = 0; i < 4; ++i) {
        memset(inst->rev_comb_l[i].buffer, 0, sizeof(inst->rev_comb_l[i].buffer));
        inst->rev_comb_l[i].size = REVERB_COMB_L[i];
        inst->rev_comb_l[i].idx = 0;
        inst->rev_comb_l[i].filter = 0.0f;

        memset(inst->rev_comb_r[i].buffer, 0, sizeof(inst->rev_comb_r[i].buffer));
        inst->rev_comb_r[i].size = REVERB_COMB_R[i];
        inst->rev_comb_r[i].idx = 0;
        inst->rev_comb_r[i].filter = 0.0f;
    }
    for (int i = 0; i < 2; ++i) {
        memset(inst->rev_ap_l[i].buffer, 0, sizeof(inst->rev_ap_l[i].buffer));
        inst->rev_ap_l[i].size = REVERB_ALLPASS_L[i];
        inst->rev_ap_l[i].idx = 0;

        memset(inst->rev_ap_r[i].buffer, 0, sizeof(inst->rev_ap_r[i].buffer));
        inst->rev_ap_r[i].size = REVERB_ALLPASS_R[i];
        inst->rev_ap_r[i].idx = 0;
    }
}

static void reverb_recompute(chordism_instance_t *inst) {
    /* Comb feedback maps decay 0..1 → 0.70..0.97 (long tail at top). */
    float fb = 0.70f + inst->reverb_decay * 0.27f;
    float dp = inst->reverb_damp * 0.5f;   /* gentle damping range */
    for (int i = 0; i < 4; ++i) {
        inst->rev_comb_l[i].feedback = fb;
        inst->rev_comb_l[i].damp = dp;
        inst->rev_comb_r[i].feedback = fb;
        inst->rev_comb_r[i].damp = dp;
    }
}

static void vibrato_recompute(chordism_instance_t *inst) {
    inst->vibrato_phase_inc = vib_speed_to_hz(inst->vib_speed) / SAMPLE_RATE;

    /* delay=0 → instant full ramp (single sample). */
    float delay_s = inst->vib_delay * VIB_DELAY_MAX_S;
    float delay_samples = delay_s * SAMPLE_RATE;
    if (delay_samples < 1.0f) delay_samples = 1.0f;
    inst->vib_ramp_inc = 1.0f / delay_samples;
}

/* Forward declarations needed because preset_apply calls these recompute
 * helpers, which are defined further down. */
static void filter_recompute(chordism_instance_t *inst);
static void sweep_recompute(chordism_instance_t *inst);
static void arp_recompute(chordism_instance_t *inst);
static void env_recompute_rates(AREnv *env, float attack01, float release01);
static void aenv_recompute_rates(ADEnv *env, float attack01, float decay01);

static void preset_apply(chordism_instance_t *inst, int idx) {
    if (idx < 0) idx = 0;
    if (idx >= NUM_PRESETS) idx = NUM_PRESETS - 1;
    const Preset *p = &PRESETS[idx];
    inst->preset_index = idx;

    inst->chord_type = p->chord_type;
    inst->chord_spread = p->chord_spread;
    inst->chord_rotation = p->chord_rotation;
    inst->detune = p->detune;
    inst->width = p->width;
    for (int i = 0; i < CHORD_SIZE; ++i) {
        inst->waveforms[i] = p->waveforms[i];
        inst->shapes[i] = p->shapes[i];
    }
    inst->morph_index = p->morph_index;
    inst->morph_intensity = p->morph_intensity;
    inst->pan_morph_index = p->pan_morph_index;
    inst->pan_morph_intensity = p->pan_morph_intensity;
    inst->fm_modulator_idx = p->fm_modulator_idx;
    inst->fm_amount = p->fm_amount;
    inst->lfo_rate = p->lfo_rate;
    inst->lfo_depth = p->lfo_depth;
    inst->lfo_shape = p->lfo_shape;
    if (inst->lfo_shape < 0) inst->lfo_shape = 0;
    if (inst->lfo_shape >= NUM_LFO_SHAPES) inst->lfo_shape = NUM_LFO_SHAPES - 1;
    inst->vib_depth = p->vib_depth;
    inst->vib_speed = p->vib_speed;
    inst->vib_delay = p->vib_delay;
    inst->sweep_amount = p->sweep_amount;
    inst->sweep_rate = p->sweep_rate;
    inst->glide_rate = p->glide_rate;
    inst->filter_cutoff = p->filter_cutoff;
    inst->filter_resonance = p->filter_resonance;
    inst->filter_mode = p->filter_mode;
    inst->filter_slope = p->filter_slope;
    inst->filter_env_attack = p->filter_env_attack;
    inst->filter_env_decay = p->filter_env_decay;
    inst->filter_env_depth = p->filter_env_depth;
    inst->filter_lfo_rate = p->filter_lfo_rate;
    inst->filter_lfo_depth = p->filter_lfo_depth;
    inst->filter_lfo_spread = p->filter_lfo_spread;
    inst->filter_lfo_shape = p->filter_lfo_shape;
    inst->drive = p->drive;
    inst->attack = p->attack;
    inst->release = p->release;
    inst->volume = p->volume;
    inst->reverb_mix = p->reverb_mix;
    inst->reverb_decay = p->reverb_decay;
    inst->reverb_damp = p->reverb_damp;
    inst->delay_mix = p->delay_mix;
    inst->delay_time = p->delay_time;
    inst->delay_feedback = p->delay_feedback;
    inst->delay_tone = p->delay_tone;
    inst->grind = p->grind;
    inst->bit_shift = p->bit_shift;
    inst->decimator = p->decimator;
    inst->arp_enabled = p->arp_enabled;
    inst->arp_tempo = p->arp_tempo;
    inst->arp_direction = p->arp_direction;
    inst->delay_mode = p->delay_mode;
    inst->vib_stray = p->vib_stray;
    inst->glide_legato = p->glide_legato;

    /* Recompute derived values. */
    morph_recompute(inst);
    pan_morph_recompute(inst);
    filter_recompute(inst);
    reverb_recompute(inst);
    vibrato_recompute(inst);
    sweep_recompute(inst);
    arp_recompute(inst);
    inst->shape_lfo.phase_inc = lfo_rate_to_hz(inst->lfo_rate) / SAMPLE_RATE;
    inst->shape_lfo.shape = inst->lfo_shape;
    inst->filter_lfo_phase_inc = lfo_rate_to_hz(inst->filter_lfo_rate) / SAMPLE_RATE;
    for (int i = 0; i < NUM_VOICES; ++i) {
        env_recompute_rates(&inst->voices[i].env, inst->attack, inst->release);
    }
    aenv_recompute_rates(&inst->filter_env,
                         inst->filter_env_attack, inst->filter_env_decay);
}

static void arp_recompute(chordism_instance_t *inst) {
    float bpm = ARP_BPM_MIN + (ARP_BPM_MAX - ARP_BPM_MIN) * inst->arp_tempo;
    /* Quarter note period in samples = 60/BPM * SR. */
    float period = (60.0f / bpm) * SAMPLE_RATE;
    int p = (int)period;
    if (p < 64) p = 64;
    inst->arp_step_period = p;
}

static void sweep_recompute(chordism_instance_t *inst) {
    /* Time exp-mapped: rate=0 → slow (4s), rate=1 → fast (10ms). */
    float ratio = SWEEP_TIME_MAX_S / SWEEP_TIME_MIN_S;
    float time_s = SWEEP_TIME_MAX_S / powf(ratio, inst->sweep_rate);
    float samples = time_s * SAMPLE_RATE;
    if (samples < 1.0f) samples = 1.0f;
    inst->sweep_coef = expf(-5.0f / samples);
}

/* Exp-map filter cutoff. */
static float filter_cutoff_to_hz(float cutoff01) {
    float ratio = FILTER_CUTOFF_MAX_HZ / FILTER_CUTOFF_MIN_HZ;
    return FILTER_CUTOFF_MIN_HZ * powf(ratio, cutoff01);
}

/* Recompute TPT SVF coefficients. Stable for any g and k. */
static void filter_recompute(chordism_instance_t *inst) {
    float hz = filter_cutoff_to_hz(inst->filter_cutoff);
    float max_hz = SAMPLE_RATE * 0.49f;
    if (hz > max_hz) hz = max_hz;
    if (hz < 1.0f) hz = 1.0f;

    float g = tanf(3.14159265358979f * hz / SAMPLE_RATE);

    /* Q ramps from broad (FILTER_Q_MIN) to resonant (FILTER_Q_MAX). */
    float Q = FILTER_Q_MIN + (FILTER_Q_MAX - FILTER_Q_MIN) * inst->filter_resonance;
    if (Q < 0.05f) Q = 0.05f;
    float k = 1.0f / Q;

    inst->filter_k  = k;
    inst->filter_a1 = 1.0f / (1.0f + g * (g + k));
    inst->filter_a2 = g * inst->filter_a1;
    inst->filter_a3 = g * inst->filter_a2;
}

/* TPT SVF: stable for any g (tan-mapped cutoff) and k (1/Q).
 * Andy Simper / Vadim Zavalishin formulation.
 * mode: FILT_LP / FILT_HP / FILT_BP. */
static inline float svf_process(SVF *s, float input, int mode,
                                float a1, float a2, float a3, float k) {
    float v3 = input - s->ic2eq;
    float v1 = a1 * s->ic1eq + a2 * v3;
    float v2 = s->ic2eq + a2 * s->ic1eq + a3 * v3;
    s->ic1eq = 2.0f * v1 - s->ic1eq;
    s->ic2eq = 2.0f * v2 - s->ic2eq;

    switch (mode) {
        case FILT_HP: return input - k * v1 - v2;
        case FILT_BP: return v1;
        case FILT_LP:
        default:      return v2;
    }
}

/* Reflective wavefolder — folds input back across [-1, +1] (Buchla style). */
static float wavefold(float x) {
    /* Iterative reflection; bounded loop count for safety. */
    for (int i = 0; i < 8; ++i) {
        if (x > 1.0f)       x = 2.0f - x;
        else if (x < -1.0f) x = -2.0f - x;
        else break;
    }
    return x;
}

static float osc_sample(int waveform, float phase, float phase_inc, float shape) {
    switch (waveform) {
        case WAVE_OFF:
            return 0.0f;

        case WAVE_SINE: {
            /* shape drives a wavefolder. gain ramps from 1 (no fold) to 6
             * (heavy fold, complex harmonics). */
            float gain = 1.0f + shape * 5.0f;
            return wavefold(gain * sinf(TWO_PI * phase));
        }

        case WAVE_TRIANGLE: {
            /* shape tilts the triangle toward sawtooth. peak position
             * moves from 0.5 (centered tri) to 1e-4 (falling saw). */
            float peak = 0.5f * (1.0f - shape);
            if (peak < 1e-4f) peak = 1e-4f;
            if (phase < peak) {
                return (phase / peak) * 2.0f - 1.0f;
            }
            return 1.0f - ((phase - peak) / (1.0f - peak)) * 2.0f;
        }

        case WAVE_SAW: {
            /* Fundamental saw plus an octave-up saw, crossfaded by shape.
             * shape=0: pure fundamental. shape=1: pure octave. */
            float s = 2.0f * phase - 1.0f;
            s -= poly_blep(phase, phase_inc);

            if (shape > 0.0f) {
                float p2 = phase * 2.0f;
                if (p2 >= 1.0f) p2 -= 1.0f;
                float s2 = 2.0f * p2 - 1.0f;
                s2 -= poly_blep(p2, phase_inc * 2.0f);
                s = s * (1.0f - shape) + s2 * shape;
            }
            return s;
        }

        case WAVE_SQUARE: {
            /* shape controls pulse width: 0 → 5%, 0.5 → 50%, 1 → 95%. */
            float pw = 0.05f + shape * 0.9f;
            float t2 = phase + (1.0f - pw);
            if (t2 >= 1.0f) t2 -= 1.0f;

            float s = (phase < pw) ? 1.0f : -1.0f;
            s += poly_blep(phase, phase_inc);
            s -= poly_blep(t2, phase_inc);
            return s;
        }

        default:
            return 0.0f;
    }
}

static void aenv_recompute_rates(ADEnv *env, float attack01, float decay01) {
    float attack_s = lerp01(attack01, ATTACK_MIN_S, ATTACK_MAX_S);
    float decay_s = lerp01(decay01, RELEASE_MIN_S, RELEASE_MAX_S);

    float attack_samples = attack_s * SAMPLE_RATE;
    if (attack_samples < 1.0f) attack_samples = 1.0f;
    env->attack_inc = 1.0f / attack_samples;

    float decay_samples = decay_s * SAMPLE_RATE;
    if (decay_samples < 1.0f) decay_samples = 1.0f;
    env->decay_coef = expf(-5.0f / decay_samples);
}

/* Tick AD envelope by one sample, return current value 0..1.
 * Stage flow: ATTACK (linear up to 1) → RELEASE (exp decay) → IDLE.
 * Reuses ENV_RELEASE as the decay stage. */
static inline float aenv_tick(ADEnv *env) {
    switch (env->stage) {
        case ENV_ATTACK:
            env->value += env->attack_inc;
            if (env->value >= 1.0f) {
                env->value = 1.0f;
                env->stage = ENV_RELEASE;
            }
            break;
        case ENV_RELEASE:
            env->value *= env->decay_coef;
            if (env->value < ENV_SILENCE) {
                env->value = 0.0f;
                env->stage = ENV_IDLE;
            }
            break;
        case ENV_IDLE:
        default:
            break;
    }
    return env->value;
}

static void env_recompute_rates(AREnv *env, float attack01, float release01) {
    float attack_s = lerp01(attack01, ATTACK_MIN_S, ATTACK_MAX_S);
    float release_s = lerp01(release01, RELEASE_MIN_S, RELEASE_MAX_S);

    float attack_samples = attack_s * SAMPLE_RATE;
    if (attack_samples < 1.0f) attack_samples = 1.0f;
    env->attack_inc = 1.0f / attack_samples;

    float release_samples = release_s * SAMPLE_RATE;
    if (release_samples < 1.0f) release_samples = 1.0f;
    env->release_coef = expf(-5.0f / release_samples);
}

/* Release every active voice. Used on note steal and on All Notes Off. */
static void release_all_voices(chordism_instance_t *inst) {
    for (int i = 0; i < NUM_VOICES; ++i) {
        Voice *v = &inst->voices[i];
        if (v->active && v->env.stage != ENV_RELEASE) {
            v->env.stage = ENV_RELEASE;
        }
    }
}

static void voice_start(chordism_instance_t *inst, Voice *v,
                        int root_note, int interval_semis,
                        float detune_cents, int velocity,
                        int chord_step) {
    v->active = true;
    v->root_note = root_note;
    v->chord_step = chord_step;
    v->waveform = inst->waveforms[chord_step];
    v->phase = 0.0f;
    float hz = midi_to_hz(root_note + interval_semis);
    if (detune_cents != 0.0f) {
        hz *= powf(2.0f, detune_cents / 1200.0f);
    }
    float target_inc = hz / SAMPLE_RATE;
    v->target_phase_inc = target_inc;

    bool glide_active = inst->has_prev_chord && inst->glide_rate > 0.0f;
    if (glide_active && inst->glide_legato && !inst->prev_chord_still_held) {
        glide_active = false;
    }
    if (glide_active) {
        float start = inst->prev_phase_inc[chord_step];
        float glide_s = inst->glide_rate * GLIDE_TIME_MAX_S;
        float glide_samples = glide_s * SAMPLE_RATE;
        if (glide_samples < 1.0f) glide_samples = 1.0f;
        v->phase_inc = start;
        v->glide_step = (target_inc - start) / glide_samples;
    } else {
        v->phase_inc = target_inc;
        v->glide_step = 0.0f;
    }

    v->velocity = (float)velocity / 127.0f;

    /* Pan offset is fixed per voice (chord position). The render loop applies
     * the live `width` knob value each sample, so pan responds in realtime. */
    float norm = (CHORD_SIZE > 1)
        ? ((float)chord_step / (float)(CHORD_SIZE - 1))   /* 0..1 across chord */
        : 0.5f;
    v->pan_offset = norm - 0.5f;

    env_recompute_rates(&v->env, inst->attack, inst->release);
    /* Reset env value so each voice starts from 0 — voice steal already
     * released the previous chord. */
    v->env.value = 0.0f;
    v->env.stage = ENV_ATTACK;
}

static void chord_on(chordism_instance_t *inst, int root_note, int velocity) {
    /* Release the previously-active chord. Its voices keep ringing through
     * their decay while the new chord starts in the *other* voice bank —
     * masks retrigger click and lets release tails breathe. */
    release_all_voices(inst);

    int base = inst->next_chord_base;
    int chord = inst->chord_type;
    if (chord < 0) chord = 0;
    if (chord >= NUM_CHORDS) chord = NUM_CHORDS - 1;
    const int *intervals = CHORD_TABLE[chord];

    /* Spread scales intervals (0 → unison, 0.5 → original, 1 → 2× wide).
     * Rotation shifts which osc plays which chord degree. */
    float spread_mult = inst->chord_spread * 2.0f;
    int rotation = (int)(inst->chord_rotation * 3.999f);
    if (rotation < 0) rotation = 0;
    if (rotation >= CHORD_SIZE) rotation = CHORD_SIZE - 1;

    for (int i = 0; i < CHORD_SIZE; ++i) {
        Voice *target = &inst->voices[base + i];
        int src = (i + rotation) % CHORD_SIZE;
        int scaled_interval = (int)((float)intervals[src] * spread_mult);
        /* Detune linear: voice 0 = 0¢, voice 1 = +detune*MAX¢, ... */
        float cents = (float)i * inst->detune * MAX_DETUNE_CENTS;
        voice_start(inst, target, root_note, scaled_interval, cents, velocity, i);
    }
    inst->next_chord_base = (base + CHORD_SIZE) % NUM_VOICES;

    /* Save this chord's per-step target as the glide source for the next chord. */
    for (int i = 0; i < CHORD_SIZE; ++i) {
        inst->prev_phase_inc[i] = inst->voices[base + i].target_phase_inc;
    }
    inst->has_prev_chord = true;

    /* Re-trigger filter envelope from the start (hard reset to 0). */
    aenv_recompute_rates(&inst->filter_env,
                         inst->filter_env_attack, inst->filter_env_decay);
    inst->filter_env.value = 0.0f;
    inst->filter_env.stage = ENV_ATTACK;

    /* Re-trigger vibrato delay ramp (phase stays continuous). */
    inst->vib_ramp_value = 0.0f;

    /* Re-trigger pitch sweep: start at full offset. */
    inst->sweep_value = inst->sweep_amount * SWEEP_MAX_SEMITONES;
}

static void chord_off(chordism_instance_t *inst, int root_note) {
    /* Release all voices whose root_note matches. (Last-note mono means
     * only the most recent chord is sounding; this handles the case where
     * the user releases the same key.) */
    for (int i = 0; i < NUM_VOICES; ++i) {
        Voice *v = &inst->voices[i];
        if (v->active && v->root_note == root_note &&
            v->env.stage != ENV_RELEASE) {
            v->env.stage = ENV_RELEASE;
        }
    }
}

/* Held-note stack helpers. */
static void held_push(chordism_instance_t *inst, int note) {
    /* Remove any previous occurrence so duplicates don't pile up. */
    for (int i = 0; i < inst->held_count; ++i) {
        if (inst->held_notes[i] == note) {
            for (int j = i; j < inst->held_count - 1; ++j) {
                inst->held_notes[j] = inst->held_notes[j + 1];
            }
            inst->held_count--;
            break;
        }
    }
    if (inst->held_count < HELD_STACK_MAX) {
        inst->held_notes[inst->held_count++] = note;
    }
}

/* Returns true if note was on top of stack (currently playing); false if it
 * was elsewhere or absent. */
static bool held_pop(chordism_instance_t *inst, int note) {
    if (inst->held_count == 0) return false;
    int top = inst->held_notes[inst->held_count - 1];
    for (int i = 0; i < inst->held_count; ++i) {
        if (inst->held_notes[i] == note) {
            for (int j = i; j < inst->held_count - 1; ++j) {
                inst->held_notes[j] = inst->held_notes[j + 1];
            }
            inst->held_count--;
            return (note == top);
        }
    }
    return false;
}

/* ------------------------------------------------------------------------- */
/* v2 API callbacks                                                          */
/* ------------------------------------------------------------------------- */

static void* v2_create_instance(const char *module_dir, const char *json_defaults) {
    (void)json_defaults;
    auto *inst = (chordism_instance_t*)calloc(1, sizeof(chordism_instance_t));
    if (!inst) return nullptr;
    if (module_dir) {
        strncpy(inst->module_dir, module_dir, sizeof(inst->module_dir) - 1);
    }

    inst->attack = 0.05f;
    inst->release = 0.30f;
    inst->volume = 0.80f;
    for (int i = 0; i < CHORD_SIZE; ++i) {
        inst->waveforms[i] = WAVE_SINE;
        inst->shapes[i] = 0.0f;
    }
    inst->morph_index = 0.0f;
    inst->morph_intensity = 0.0f;
    morph_recompute(inst);
    inst->pan_morph_index = 0.0f;
    inst->pan_morph_intensity = 0.0f;
    pan_morph_recompute(inst);
    inst->chord_spread = 0.5f;       /* 1.0× multiplier */
    inst->chord_rotation = 0.0f;
    inst->fm_modulator_idx = 0;
    inst->fm_amount = 0.0f;
    inst->last_modulator_sample = 0.0f;
    inst->lfo_rate = 0.0f;
    inst->lfo_depth = 0.0f;
    inst->lfo_shape = LFO_TRIANGLE;
    inst->chord_type = CHORD_MAJOR;
    inst->detune = 0.0f;
    inst->width = 1.0f;               /* full stereo spread by default */
    inst->vib_speed = 0.5f;           /* ~1 Hz */
    inst->vib_depth = 0.0f;           /* disabled by default */
    inst->vib_delay = 0.2f;
    inst->vibrato_phase = 0.0f;
    inst->vib_ramp_value = 0.0f;
    vibrato_recompute(inst);
    inst->sweep_amount = 0.0f;
    inst->sweep_rate = 0.5f;
    inst->sweep_value = 0.0f;
    sweep_recompute(inst);
    inst->held_count = 0;
    inst->aftertouch = 0.0f;
    inst->glide_rate = 0.0f;
    inst->glide_legato = 0;
    inst->has_prev_chord = false;
    inst->prev_chord_still_held = false;
    for (int i = 0; i < CHORD_SIZE; ++i) inst->prev_phase_inc[i] = 0.0f;
    inst->vib_stray = 0;
    inst->vib_random_value = 0.0f;
    inst->vib_random_target = 0.0f;
    inst->vib_random_counter = 0;
    inst->preset_index = 0;
    preset_apply(inst, 0);  /* Init preset */
    inst->arp_enabled = 0;
    inst->arp_tempo = 0.4f;            /* ~120 BPM */
    inst->arp_direction = ARP_UP;
    inst->arp_step_idx = 0;
    inst->arp_step_dir = 1;
    inst->arp_sample_counter = 0;
    arp_recompute(inst);
    inst->reverb_mix = 0.0f;          /* dry by default */
    inst->reverb_decay = 0.5f;
    inst->reverb_damp = 0.3f;
    reverb_init(inst);
    reverb_recompute(inst);
    inst->grind = 0.0f;
    inst->bit_shift = 0.0f;
    inst->decimator = 0.0f;
    inst->decim_counter = 0;
    inst->decim_hold_l = 0.0f;
    inst->decim_hold_r = 0.0f;
    inst->delay_mix = 0.0f;
    inst->delay_time = 0.3f;
    inst->delay_feedback = 0.4f;
    inst->delay_tone = 0.7f;
    inst->delay_mode = DELAY_STEREO;
    inst->delay_write_idx = 0;
    inst->delay_lp_l = 0.0f;
    inst->delay_lp_r = 0.0f;
    inst->delay_flip_counter = 0;
    memset(inst->delay_buf_l, 0, sizeof(inst->delay_buf_l));
    memset(inst->delay_buf_r, 0, sizeof(inst->delay_buf_r));
    inst->filter_cutoff = 1.0f;       /* wide open by default */
    inst->filter_resonance = 0.0f;    /* no resonance */
    inst->filter_mode = FILT_LP;
    inst->filter_slope = 0;           /* 12 dB by default */
    inst->filter_env_attack = 0.0f;
    inst->filter_env_decay = 0.30f;
    inst->filter_env_depth = 0.0f;    /* disabled by default */
    inst->filter_env.stage = ENV_IDLE;
    inst->filter_env.value = 0.0f;
    aenv_recompute_rates(&inst->filter_env,
                         inst->filter_env_attack, inst->filter_env_decay);
    inst->filter_l_b.ic1eq = 0.0f;
    inst->filter_l_b.ic2eq = 0.0f;
    inst->filter_r_b.ic1eq = 0.0f;
    inst->filter_r_b.ic2eq = 0.0f;
    inst->filter_lfo_rate = 0.0f;
    inst->filter_lfo_depth = 0.0f;
    inst->filter_lfo_spread = 0.0f;
    inst->filter_lfo_shape = LFO_TRIANGLE;
    inst->filter_lfo_phase = 0.0f;
    inst->filter_lfo_phase_inc = lfo_rate_to_hz(inst->filter_lfo_rate) / SAMPLE_RATE;
    inst->drive = 0.0f;               /* clean by default */
    inst->filter_l.ic1eq = 0.0f;
    inst->filter_l.ic2eq = 0.0f;
    inst->filter_r.ic1eq = 0.0f;
    inst->filter_r.ic2eq = 0.0f;
    filter_recompute(inst);
    inst->shape_lfo.phase = 0.0f;
    inst->shape_lfo.phase_inc = lfo_rate_to_hz(inst->lfo_rate) / SAMPLE_RATE;
    inst->shape_lfo.shape = inst->lfo_shape;

    for (int i = 0; i < NUM_VOICES; ++i) {
        inst->voices[i].env.stage = ENV_IDLE;
        env_recompute_rates(&inst->voices[i].env, inst->attack, inst->release);
    }

    if (g_host && g_host->log) {
        g_host->log("[chordism] instance created (4-voice)");
    }
    return inst;
}

static void v2_destroy_instance(void *instance) {
    free(instance);
}

static void v2_on_midi(void *instance, const uint8_t *msg, int len, int source) {
    (void)source;
    if (!instance || !msg || len < 3) return;
    auto *inst = (chordism_instance_t*)instance;

    uint8_t status = msg[0] & 0xF0;
    uint8_t d1 = msg[1] & 0x7F;
    uint8_t d2 = msg[2] & 0x7F;

    if (status == 0x90 && d2 > 0) {
        /* Legato flag: a previous chord is still ringing if any pad is held.
         * Capture this BEFORE pushing the new note to the stack. */
        inst->prev_chord_still_held = (inst->held_count > 0);
        held_push(inst, d1);
        if (inst->arp_enabled) {
            /* Fire this note immediately, then schedule next arp tick. */
            chord_on(inst, d1, d2);
            inst->arp_step_idx = inst->held_count - 1;
            inst->arp_step_dir = 1;
            inst->arp_sample_counter = inst->arp_step_period;
        } else {
            chord_on(inst, d1, d2);
        }
    } else if (status == 0x80 || (status == 0x90 && d2 == 0)) {
        bool was_top = held_pop(inst, d1);
        if (inst->arp_enabled) {
            /* Arp keeps cycling — don't retrigger here. If stack empties,
             * arp will simply stop firing until next note-on. */
            if (inst->held_count == 0) {
                chord_off(inst, d1);
            }
        } else if (was_top) {
            /* Mono-priority return to previously-held. */
            if (inst->held_count > 0) {
                int prev = inst->held_notes[inst->held_count - 1];
                chord_on(inst, prev, 100);
            } else {
                chord_off(inst, d1);
            }
        }
    } else if (status == 0xD0) {
        /* Channel aftertouch — single-byte pressure in d1. */
        inst->aftertouch = (float)d1 / 127.0f;
    } else if (status == 0xB0 && d1 == 123) {
        inst->held_count = 0;
        release_all_voices(inst);
    }
}

static void v2_set_param(void *instance, const char *key, const char *val) {
    if (!instance || !key) return;
    auto *inst = (chordism_instance_t*)instance;

    if (strcmp(key, "preset") == 0) {
        if (val) {
            int p = atoi(val);
            if (p < 0) p = 0;
            if (p >= NUM_PRESETS) p = NUM_PRESETS - 1;
            preset_apply(inst, p);
        }
        return;
    }

    if (strcmp(key, "attack") == 0) {
        inst->attack = param_from_string(val, inst->attack);
        for (int i = 0; i < NUM_VOICES; ++i) {
            env_recompute_rates(&inst->voices[i].env, inst->attack, inst->release);
        }
    } else if (strcmp(key, "release") == 0) {
        inst->release = param_from_string(val, inst->release);
        for (int i = 0; i < NUM_VOICES; ++i) {
            env_recompute_rates(&inst->voices[i].env, inst->attack, inst->release);
        }
    } else if (strcmp(key, "volume") == 0) {
        inst->volume = param_from_string(val, inst->volume);
    } else if (strcmp(key, "waveform") == 0) {
        /* Compatibility: "waveform" sets ALL chord steps to the same value. */
        if (val) {
            int w = atoi(val);
            if (w < 0) w = 0;
            if (w >= NUM_WAVEFORMS) w = NUM_WAVEFORMS - 1;
            for (int i = 0; i < CHORD_SIZE; ++i) inst->waveforms[i] = w;
        }
    } else if (strncmp(key, "wave_", 5) == 0 && key[5] >= '1' && key[5] <= '0' + CHORD_SIZE) {
        int idx = key[5] - '1';
        if (val) {
            int w = atoi(val);
            if (w < 0) w = 0;
            if (w >= NUM_WAVEFORMS) w = NUM_WAVEFORMS - 1;
            inst->waveforms[idx] = w;
        }
    } else if (strcmp(key, "shape") == 0) {
        /* Compatibility: sets ALL per-osc shapes. */
        float v = param_from_string(val, inst->shapes[0]);
        for (int i = 0; i < CHORD_SIZE; ++i) inst->shapes[i] = v;
    } else if (strncmp(key, "shape_", 6) == 0 && key[6] >= '1' && key[6] <= '0' + CHORD_SIZE && key[7] == '\0') {
        int idx = key[6] - '1';
        inst->shapes[idx] = param_from_string(val, inst->shapes[idx]);
    } else if (strcmp(key, "pan_morph_index") == 0) {
        inst->pan_morph_index = param_from_string(val, inst->pan_morph_index);
        pan_morph_recompute(inst);
    } else if (strcmp(key, "pan_morph_intensity") == 0) {
        inst->pan_morph_intensity = param_from_string(val, inst->pan_morph_intensity);
        pan_morph_recompute(inst);
    } else if (strcmp(key, "chord_spread") == 0) {
        inst->chord_spread = param_from_string(val, inst->chord_spread);
    } else if (strcmp(key, "chord_rotation") == 0) {
        inst->chord_rotation = param_from_string(val, inst->chord_rotation);
    } else if (strcmp(key, "fm_modulator") == 0) {
        if (val) {
            int m = atoi(val);
            if (m < 0) m = 0;
            if (m >= CHORD_SIZE) m = CHORD_SIZE - 1;
            inst->fm_modulator_idx = m;
        }
    } else if (strcmp(key, "fm_amount") == 0) {
        inst->fm_amount = param_from_string(val, inst->fm_amount);
    } else if (strcmp(key, "lfo_rate") == 0) {
        inst->lfo_rate = param_from_string(val, inst->lfo_rate);
        inst->shape_lfo.phase_inc = lfo_rate_to_hz(inst->lfo_rate) / SAMPLE_RATE;
    } else if (strcmp(key, "lfo_depth") == 0) {
        inst->lfo_depth = param_from_string(val, inst->lfo_depth);
    } else if (strcmp(key, "lfo_shape") == 0) {
        if (val) {
            int s = atoi(val);
            if (s < 0) s = 0;
            if (s >= NUM_LFO_SHAPES) s = NUM_LFO_SHAPES - 1;
            inst->lfo_shape = s;
            inst->shape_lfo.shape = s;
        }
    } else if (strcmp(key, "chord_type") == 0) {
        if (val) {
            int c = atoi(val);
            if (c < 0) c = 0;
            if (c >= NUM_CHORDS) c = NUM_CHORDS - 1;
            inst->chord_type = c;
        }
    } else if (strcmp(key, "detune") == 0) {
        inst->detune = param_from_string(val, inst->detune);
    } else if (strcmp(key, "width") == 0) {
        inst->width = param_from_string(val, inst->width);
    } else if (strcmp(key, "vib_speed") == 0) {
        inst->vib_speed = param_from_string(val, inst->vib_speed);
        vibrato_recompute(inst);
    } else if (strcmp(key, "vib_depth") == 0) {
        inst->vib_depth = param_from_string(val, inst->vib_depth);
    } else if (strcmp(key, "vib_delay") == 0) {
        inst->vib_delay = param_from_string(val, inst->vib_delay);
        vibrato_recompute(inst);
    } else if (strcmp(key, "sweep_amount") == 0) {
        if (val) {
            char *end = nullptr;
            float v = strtof(val, &end);
            if (end != val) {
                if (v < -1.0f) v = -1.0f;
                if (v > 1.0f) v = 1.0f;
                inst->sweep_amount = v;
            }
        }
    } else if (strcmp(key, "sweep_rate") == 0) {
        inst->sweep_rate = param_from_string(val, inst->sweep_rate);
        sweep_recompute(inst);
    } else if (strcmp(key, "filter_cutoff") == 0) {
        inst->filter_cutoff = param_from_string(val, inst->filter_cutoff);
        filter_recompute(inst);
    } else if (strcmp(key, "filter_resonance") == 0) {
        inst->filter_resonance = param_from_string(val, inst->filter_resonance);
        filter_recompute(inst);
    } else if (strcmp(key, "drive") == 0) {
        inst->drive = param_from_string(val, inst->drive);
    } else if (strcmp(key, "filter_mode") == 0) {
        if (val) {
            int m = atoi(val);
            if (m < 0) m = 0;
            if (m >= NUM_FILTER_MODES) m = NUM_FILTER_MODES - 1;
            inst->filter_mode = m;
        }
    } else if (strcmp(key, "filter_slope") == 0) {
        if (val) {
            int s = atoi(val);
            inst->filter_slope = s ? 1 : 0;
        }
    } else if (strcmp(key, "filter_lfo_rate") == 0) {
        inst->filter_lfo_rate = param_from_string(val, inst->filter_lfo_rate);
        inst->filter_lfo_phase_inc = lfo_rate_to_hz(inst->filter_lfo_rate) / SAMPLE_RATE;
    } else if (strcmp(key, "filter_lfo_depth") == 0) {
        inst->filter_lfo_depth = param_from_string(val, inst->filter_lfo_depth);
    } else if (strcmp(key, "filter_lfo_spread") == 0) {
        inst->filter_lfo_spread = param_from_string(val, inst->filter_lfo_spread);
    } else if (strcmp(key, "filter_lfo_shape") == 0) {
        if (val) {
            int s = atoi(val);
            if (s < 0) s = 0;
            if (s >= NUM_LFO_SHAPES) s = NUM_LFO_SHAPES - 1;
            inst->filter_lfo_shape = s;
        }
    } else if (strcmp(key, "filter_env_attack") == 0) {
        inst->filter_env_attack = param_from_string(val, inst->filter_env_attack);
        aenv_recompute_rates(&inst->filter_env,
                             inst->filter_env_attack, inst->filter_env_decay);
    } else if (strcmp(key, "filter_env_decay") == 0) {
        inst->filter_env_decay = param_from_string(val, inst->filter_env_decay);
        aenv_recompute_rates(&inst->filter_env,
                             inst->filter_env_attack, inst->filter_env_decay);
    } else if (strcmp(key, "filter_env_depth") == 0) {
        if (val) {
            char *end = nullptr;
            float v = strtof(val, &end);
            if (end != val) {
                if (v < -1.0f) v = -1.0f;
                if (v > 1.0f) v = 1.0f;
                inst->filter_env_depth = v;
            }
        }
    } else if (strcmp(key, "reverb_mix") == 0) {
        inst->reverb_mix = param_from_string(val, inst->reverb_mix);
    } else if (strcmp(key, "reverb_decay") == 0) {
        inst->reverb_decay = param_from_string(val, inst->reverb_decay);
        reverb_recompute(inst);
    } else if (strcmp(key, "reverb_damp") == 0) {
        inst->reverb_damp = param_from_string(val, inst->reverb_damp);
        reverb_recompute(inst);
    } else if (strcmp(key, "grind") == 0) {
        inst->grind = param_from_string(val, inst->grind);
    } else if (strcmp(key, "bit_shift") == 0) {
        inst->bit_shift = param_from_string(val, inst->bit_shift);
    } else if (strcmp(key, "decimator") == 0) {
        inst->decimator = param_from_string(val, inst->decimator);
    } else if (strcmp(key, "morph_index") == 0) {
        inst->morph_index = param_from_string(val, inst->morph_index);
        morph_recompute(inst);
    } else if (strcmp(key, "morph_intensity") == 0) {
        inst->morph_intensity = param_from_string(val, inst->morph_intensity);
        morph_recompute(inst);
    } else if (strcmp(key, "delay_mix") == 0) {
        inst->delay_mix = param_from_string(val, inst->delay_mix);
    } else if (strcmp(key, "delay_time") == 0) {
        inst->delay_time = param_from_string(val, inst->delay_time);
    } else if (strcmp(key, "delay_feedback") == 0) {
        inst->delay_feedback = param_from_string(val, inst->delay_feedback);
    } else if (strcmp(key, "delay_tone") == 0) {
        inst->delay_tone = param_from_string(val, inst->delay_tone);
    } else if (strcmp(key, "delay_mode") == 0) {
        if (val) {
            int m = atoi(val);
            if (m < 0) m = 0;
            if (m >= NUM_DELAY_MODES) m = NUM_DELAY_MODES - 1;
            inst->delay_mode = m;
        }
    } else if (strcmp(key, "glide_rate") == 0) {
        inst->glide_rate = param_from_string(val, inst->glide_rate);
    } else if (strcmp(key, "glide_legato") == 0) {
        if (val) inst->glide_legato = atoi(val) ? 1 : 0;
    } else if (strcmp(key, "vib_stray") == 0) {
        if (val) inst->vib_stray = atoi(val) ? 1 : 0;
    } else if (strcmp(key, "arp_enabled") == 0) {
        if (val) {
            int e = atoi(val);
            inst->arp_enabled = e ? 1 : 0;
            if (!inst->arp_enabled) {
                inst->arp_sample_counter = 0;
            }
        }
    } else if (strcmp(key, "arp_tempo") == 0) {
        inst->arp_tempo = param_from_string(val, inst->arp_tempo);
        arp_recompute(inst);
    } else if (strcmp(key, "arp_direction") == 0) {
        if (val) {
            int d = atoi(val);
            if (d < 0) d = 0;
            if (d >= NUM_ARP_DIRECTIONS) d = NUM_ARP_DIRECTIONS - 1;
            inst->arp_direction = d;
            inst->arp_step_dir = 1;
        }
    }
}

/* Shadow UI menu structure — fetched by host via get_param("ui_hierarchy").
 * String entries in `params` are key references; objects are level nav. */
static const char *ui_hierarchy_json =
"{"
  "\"modes\":null,"
  "\"levels\":{"
    "\"root\":{"
      "\"name\":\"Chordism\","
      "\"list_param\":\"preset\","
      "\"count_param\":\"preset_count\","
      "\"name_param\":\"preset_name\","
      "\"children\":null,"
      "\"knobs\":[\"chord_type\",\"width\",\"filter_cutoff\",\"filter_resonance\",\"drive\",\"shape\",\"reverb_mix\",\"volume\"],"
      "\"params\":["
        "\"chord_type\",\"width\",\"filter_cutoff\",\"filter_resonance\","
        "\"drive\",\"shape\",\"reverb_mix\",\"volume\","
        "{\"level\":\"osc\",\"label\":\"Oscillators\"},"
        "{\"level\":\"filter\",\"label\":\"Filter\"},"
        "{\"level\":\"mod\",\"label\":\"Modulation\"},"
        "{\"level\":\"env\",\"label\":\"Envelope\"},"
        "{\"level\":\"fx\",\"label\":\"FX\"},"
        "{\"level\":\"delay\",\"label\":\"Delay\"},"
        "{\"level\":\"arp\",\"label\":\"Arp\"}"
      "]"
    "},"
    "\"osc\":{"
      "\"name\":\"Oscillators\","
      "\"children\":null,"
      "\"knobs\":[\"wave_1\",\"wave_2\",\"wave_3\",\"wave_4\",\"shape_1\",\"shape_2\",\"shape_3\",\"shape_4\"],"
      "\"params\":[\"wave_1\",\"wave_2\",\"wave_3\",\"wave_4\",\"shape_1\",\"shape_2\",\"shape_3\",\"shape_4\",\"chord_type\",\"chord_spread\",\"chord_rotation\",\"detune\",\"width\","
                  "\"morph_index\",\"morph_intensity\",\"pan_morph_index\",\"pan_morph_intensity\",\"fm_modulator\",\"fm_amount\"],"
      "\"navigate_to\":\"root\""
    "},"
    "\"filter\":{"
      "\"name\":\"Filter\","
      "\"children\":null,"
      "\"knobs\":[\"filter_cutoff\",\"filter_resonance\",\"filter_mode\",\"filter_slope\",\"filter_env_attack\",\"filter_env_decay\",\"filter_env_depth\",\"drive\"],"
      "\"params\":[\"filter_cutoff\",\"filter_resonance\",\"filter_mode\",\"filter_slope\",\"filter_env_attack\",\"filter_env_decay\",\"filter_env_depth\",\"filter_lfo_rate\",\"filter_lfo_depth\",\"filter_lfo_spread\",\"filter_lfo_shape\",\"drive\"],"
      "\"navigate_to\":\"root\""
    "},"
    "\"mod\":{"
      "\"name\":\"Modulation\","
      "\"children\":null,"
      "\"knobs\":[\"lfo_shape\",\"lfo_rate\",\"lfo_depth\",\"vib_depth\",\"vib_speed\",\"vib_stray\",\"sweep_amount\",\"glide_rate\"],"
      "\"params\":[\"lfo_shape\",\"lfo_rate\",\"lfo_depth\",\"vib_depth\",\"vib_speed\",\"vib_delay\",\"vib_stray\",\"sweep_amount\",\"sweep_rate\",\"glide_rate\",\"glide_legato\",\"detune\"],"
      "\"navigate_to\":\"root\""
    "},"
    "\"env\":{"
      "\"name\":\"Envelope\","
      "\"children\":null,"
      "\"knobs\":[\"attack\",\"release\",\"volume\"],"
      "\"params\":[\"attack\",\"release\",\"volume\"],"
      "\"navigate_to\":\"root\""
    "},"
    "\"fx\":{"
      "\"name\":\"FX\","
      "\"children\":null,"
      "\"knobs\":[\"reverb_mix\",\"reverb_decay\",\"reverb_damp\",\"grind\",\"bit_shift\",\"decimator\"],"
      "\"params\":[\"reverb_mix\",\"reverb_decay\",\"reverb_damp\",\"grind\",\"bit_shift\",\"decimator\"],"
      "\"navigate_to\":\"root\""
    "},"
    "\"delay\":{"
      "\"name\":\"Delay\","
      "\"children\":null,"
      "\"knobs\":[\"delay_mix\",\"delay_time\",\"delay_feedback\",\"delay_tone\",\"delay_mode\"],"
      "\"params\":[\"delay_mix\",\"delay_time\",\"delay_feedback\",\"delay_tone\",\"delay_mode\"],"
      "\"navigate_to\":\"root\""
    "},"
    "\"arp\":{"
      "\"name\":\"Arp\","
      "\"children\":null,"
      "\"knobs\":[\"arp_enabled\",\"arp_tempo\",\"arp_direction\"],"
      "\"params\":[\"arp_enabled\",\"arp_tempo\",\"arp_direction\"],"
      "\"navigate_to\":\"root\""
    "}"
  "}"
"}";

/* Param metadata for chain_params query — flat array, one entry per param. */
static const char *chain_params_json =
"["
  "{\"key\":\"chord_type\",\"name\":\"Chord\",\"type\":\"enum\",\"options\":[\"Octaves\",\"Fifth\",\"Minor\",\"Min 7\",\"Min 9\",\"Min 11\",\"Major\",\"Maj 7\",\"Maj 9\",\"Sus 4\",\"6/9\",\"Min 6\",\"10th\",\"Dom 7\",\"Dom 7 b9\",\"Half Dim\"],\"default\":6},"
  "{\"key\":\"detune\",\"name\":\"Detune\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"width\",\"name\":\"Width\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":1},"
  "{\"key\":\"filter_cutoff\",\"name\":\"Cutoff\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":1},"
  "{\"key\":\"filter_resonance\",\"name\":\"Reso\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"filter_mode\",\"name\":\"Mode\",\"type\":\"enum\",\"options\":[\"LP\",\"HP\",\"BP\"],\"default\":0},"
  "{\"key\":\"filter_slope\",\"name\":\"Slope\",\"type\":\"enum\",\"options\":[\"12 dB\",\"24 dB\"],\"default\":0},"
  "{\"key\":\"filter_lfo_rate\",\"name\":\"FLFO Rate\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"filter_lfo_depth\",\"name\":\"FLFO Dpt\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"filter_lfo_spread\",\"name\":\"FLFO Spd\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"filter_lfo_shape\",\"name\":\"FLFO Wave\",\"type\":\"enum\",\"options\":[\"Triangle\",\"Ramp Up\",\"Ramp Down\",\"Square\"],\"default\":0},"
  "{\"key\":\"filter_env_attack\",\"name\":\"Env A\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"filter_env_decay\",\"name\":\"Env D\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.3},"
  "{\"key\":\"filter_env_depth\",\"name\":\"Env Amt\",\"type\":\"float\",\"min\":-1,\"max\":1,\"step\":0.02,\"default\":0},"
  "{\"key\":\"drive\",\"name\":\"Drive\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"volume\",\"name\":\"Volume\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.02,\"default\":0.8},"
  "{\"key\":\"wave_1\",\"name\":\"Wave 1\",\"type\":\"enum\",\"options\":[\"Off\",\"Sine\",\"Triangle\",\"Saw\",\"Square\"],\"default\":1},"
  "{\"key\":\"wave_2\",\"name\":\"Wave 2\",\"type\":\"enum\",\"options\":[\"Off\",\"Sine\",\"Triangle\",\"Saw\",\"Square\"],\"default\":1},"
  "{\"key\":\"wave_3\",\"name\":\"Wave 3\",\"type\":\"enum\",\"options\":[\"Off\",\"Sine\",\"Triangle\",\"Saw\",\"Square\"],\"default\":1},"
  "{\"key\":\"wave_4\",\"name\":\"Wave 4\",\"type\":\"enum\",\"options\":[\"Off\",\"Sine\",\"Triangle\",\"Saw\",\"Square\"],\"default\":1},"
  "{\"key\":\"shape\",\"name\":\"Shape\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"shape_1\",\"name\":\"Shape 1\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"shape_2\",\"name\":\"Shape 2\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"shape_3\",\"name\":\"Shape 3\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"shape_4\",\"name\":\"Shape 4\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"morph_index\",\"name\":\"Morph\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"morph_intensity\",\"name\":\"Morph Int\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"pan_morph_index\",\"name\":\"Pan Morph\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"pan_morph_intensity\",\"name\":\"Pan Int\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"chord_spread\",\"name\":\"Spread\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.5},"
  "{\"key\":\"chord_rotation\",\"name\":\"Rotation\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"fm_modulator\",\"name\":\"FM Mod\",\"type\":\"int\",\"min\":0,\"max\":3,\"step\":1,\"default\":0},"
  "{\"key\":\"fm_amount\",\"name\":\"FM Amt\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"lfo_shape\",\"name\":\"LFO Wave\",\"type\":\"enum\",\"options\":[\"Triangle\",\"Ramp Up\",\"Ramp Down\",\"Square\"],\"default\":0},"
  "{\"key\":\"lfo_rate\",\"name\":\"LFO Rate\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"lfo_depth\",\"name\":\"LFO Dpt\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"vib_depth\",\"name\":\"Vib Dpt\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"vib_speed\",\"name\":\"Vib Spd\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.5},"
  "{\"key\":\"vib_delay\",\"name\":\"Vib Dly\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.2},"
  "{\"key\":\"sweep_amount\",\"name\":\"Sweep\",\"type\":\"float\",\"min\":-1,\"max\":1,\"step\":0.02,\"default\":0},"
  "{\"key\":\"sweep_rate\",\"name\":\"Swp Rate\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.5},"
  "{\"key\":\"attack\",\"name\":\"Attack\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.05},"
  "{\"key\":\"release\",\"name\":\"Release\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.3},"
  "{\"key\":\"reverb_mix\",\"name\":\"Verb Mix\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"reverb_decay\",\"name\":\"Verb Dec\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.5},"
  "{\"key\":\"reverb_damp\",\"name\":\"Verb Damp\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.3},"
  "{\"key\":\"grind\",\"name\":\"Grind\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"bit_shift\",\"name\":\"Shift\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"decimator\",\"name\":\"Decim\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"delay_mix\",\"name\":\"Dly Mix\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"delay_time\",\"name\":\"Dly Time\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.3},"
  "{\"key\":\"delay_feedback\",\"name\":\"Dly Fbk\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.4},"
  "{\"key\":\"delay_tone\",\"name\":\"Dly Tone\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.7},"
  "{\"key\":\"glide_rate\",\"name\":\"Glide\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0},"
  "{\"key\":\"glide_legato\",\"name\":\"Glide Lega\",\"type\":\"enum\",\"options\":[\"Always\",\"Legato\"],\"default\":0},"
  "{\"key\":\"vib_stray\",\"name\":\"Vib Stray\",\"type\":\"enum\",\"options\":[\"LFO\",\"Random\"],\"default\":0},"
  "{\"key\":\"delay_mode\",\"name\":\"Dly Mode\",\"type\":\"enum\",\"options\":[\"Stereo\",\"Ping-Pong\",\"Flip-Flop\"],\"default\":0},"
  "{\"key\":\"arp_enabled\",\"name\":\"Arp\",\"type\":\"enum\",\"options\":[\"Off\",\"On\"],\"default\":0},"
  "{\"key\":\"arp_tempo\",\"name\":\"Arp Tempo\",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":0.4},"
  "{\"key\":\"arp_direction\",\"name\":\"Arp Dir\",\"type\":\"enum\",\"options\":[\"Up\",\"Down\",\"Up/Down\",\"Random\"],\"default\":0}"
"]";

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len) {
    if (!instance || !buf || buf_len <= 0) return 0;
    auto *inst = (chordism_instance_t*)instance;

    /* Preset browser queries. */
    if (key && strcmp(key, "preset") == 0) {
        return snprintf(buf, buf_len, "%d", inst->preset_index);
    }
    if (key && strcmp(key, "preset_count") == 0) {
        return snprintf(buf, buf_len, "%d", NUM_PRESETS);
    }
    if (key && strcmp(key, "preset_name") == 0) {
        int idx = inst->preset_index;
        if (idx < 0) idx = 0;
        if (idx >= NUM_PRESETS) idx = NUM_PRESETS - 1;
        return snprintf(buf, buf_len, "%s", PRESETS[idx].name);
    }

    /* Metadata queries from the Shadow UI. */
    if (key && strcmp(key, "ui_hierarchy") == 0) {
        int len = (int)strlen(ui_hierarchy_json);
        if (len >= buf_len) return -1;
        memcpy(buf, ui_hierarchy_json, len + 1);
        return len;
    }
    if (key && strcmp(key, "chain_params") == 0) {
        int len = (int)strlen(chain_params_json);
        if (len >= buf_len) return -1;
        memcpy(buf, chain_params_json, len + 1);
        return len;
    }

    if (key && strcmp(key, "attack") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->attack);
    } else if (key && strcmp(key, "release") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->release);
    } else if (key && strcmp(key, "volume") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->volume);
    } else if (key && strcmp(key, "waveform") == 0) {
        /* Report voice-0 waveform for backward compatibility. */
        return snprintf(buf, buf_len, "%d", inst->waveforms[0]);
    } else if (key && strncmp(key, "wave_", 5) == 0 && key[5] >= '1' && key[5] <= '0' + CHORD_SIZE && key[6] == '\0') {
        int idx = key[5] - '1';
        return snprintf(buf, buf_len, "%d", inst->waveforms[idx]);
    } else if (key && strcmp(key, "shape") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->shapes[0]);
    } else if (key && strncmp(key, "shape_", 6) == 0 && key[6] >= '1' && key[6] <= '0' + CHORD_SIZE && key[7] == '\0') {
        int idx = key[6] - '1';
        return snprintf(buf, buf_len, "%.4f", inst->shapes[idx]);
    } else if (key && strcmp(key, "pan_morph_index") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->pan_morph_index);
    } else if (key && strcmp(key, "pan_morph_intensity") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->pan_morph_intensity);
    } else if (key && strcmp(key, "chord_spread") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->chord_spread);
    } else if (key && strcmp(key, "chord_rotation") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->chord_rotation);
    } else if (key && strcmp(key, "fm_modulator") == 0) {
        return snprintf(buf, buf_len, "%d", inst->fm_modulator_idx);
    } else if (key && strcmp(key, "fm_amount") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->fm_amount);
    } else if (key && strcmp(key, "lfo_rate") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->lfo_rate);
    } else if (key && strcmp(key, "lfo_depth") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->lfo_depth);
    } else if (key && strcmp(key, "lfo_shape") == 0) {
        return snprintf(buf, buf_len, "%d", inst->lfo_shape);
    } else if (key && strcmp(key, "chord_type") == 0) {
        return snprintf(buf, buf_len, "%d", inst->chord_type);
    } else if (key && strcmp(key, "detune") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->detune);
    } else if (key && strcmp(key, "width") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->width);
    } else if (key && strcmp(key, "vib_speed") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->vib_speed);
    } else if (key && strcmp(key, "vib_depth") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->vib_depth);
    } else if (key && strcmp(key, "vib_delay") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->vib_delay);
    } else if (key && strcmp(key, "sweep_amount") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->sweep_amount);
    } else if (key && strcmp(key, "sweep_rate") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->sweep_rate);
    } else if (key && strcmp(key, "filter_cutoff") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_cutoff);
    } else if (key && strcmp(key, "filter_resonance") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_resonance);
    } else if (key && strcmp(key, "drive") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->drive);
    } else if (key && strcmp(key, "filter_mode") == 0) {
        return snprintf(buf, buf_len, "%d", inst->filter_mode);
    } else if (key && strcmp(key, "filter_slope") == 0) {
        return snprintf(buf, buf_len, "%d", inst->filter_slope);
    } else if (key && strcmp(key, "filter_lfo_rate") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_lfo_rate);
    } else if (key && strcmp(key, "filter_lfo_depth") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_lfo_depth);
    } else if (key && strcmp(key, "filter_lfo_spread") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_lfo_spread);
    } else if (key && strcmp(key, "filter_lfo_shape") == 0) {
        return snprintf(buf, buf_len, "%d", inst->filter_lfo_shape);
    } else if (key && strcmp(key, "filter_env_attack") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_env_attack);
    } else if (key && strcmp(key, "filter_env_decay") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_env_decay);
    } else if (key && strcmp(key, "filter_env_depth") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_env_depth);
    } else if (key && strcmp(key, "reverb_mix") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->reverb_mix);
    } else if (key && strcmp(key, "reverb_decay") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->reverb_decay);
    } else if (key && strcmp(key, "reverb_damp") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->reverb_damp);
    } else if (key && strcmp(key, "grind") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->grind);
    } else if (key && strcmp(key, "bit_shift") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->bit_shift);
    } else if (key && strcmp(key, "decimator") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->decimator);
    } else if (key && strcmp(key, "morph_index") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->morph_index);
    } else if (key && strcmp(key, "morph_intensity") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->morph_intensity);
    } else if (key && strcmp(key, "delay_mix") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->delay_mix);
    } else if (key && strcmp(key, "delay_time") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->delay_time);
    } else if (key && strcmp(key, "delay_feedback") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->delay_feedback);
    } else if (key && strcmp(key, "delay_tone") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->delay_tone);
    } else if (key && strcmp(key, "delay_mode") == 0) {
        return snprintf(buf, buf_len, "%d", inst->delay_mode);
    } else if (key && strcmp(key, "glide_legato") == 0) {
        return snprintf(buf, buf_len, "%d", inst->glide_legato);
    } else if (key && strcmp(key, "vib_stray") == 0) {
        return snprintf(buf, buf_len, "%d", inst->vib_stray);
    } else if (key && strcmp(key, "glide_rate") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->glide_rate);
    } else if (key && strcmp(key, "arp_enabled") == 0) {
        return snprintf(buf, buf_len, "%d", inst->arp_enabled);
    } else if (key && strcmp(key, "arp_tempo") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->arp_tempo);
    } else if (key && strcmp(key, "arp_direction") == 0) {
        return snprintf(buf, buf_len, "%d", inst->arp_direction);
    } else if (key && strcmp(key, "version") == 0) {
        return snprintf(buf, buf_len, "0.2.2");
    }
    buf[0] = '\0';
    return 0;
}

static int v2_get_error(void *instance, char *buf, int buf_len) {
    (void)instance;
    if (buf && buf_len > 0) buf[0] = '\0';
    return 0;
}

static void v2_render_block(void *instance, int16_t *out_interleaved_lr, int frames) {
    if (!instance || !out_interleaved_lr || frames <= 0) {
        if (out_interleaved_lr && frames > 0) {
            memset(out_interleaved_lr, 0, sizeof(int16_t) * 2 * frames);
        }
        return;
    }
    auto *inst = (chordism_instance_t*)instance;

    /* Sum across voices. A chord is CHORD_SIZE voices at unit amplitude → peak
     * ±CHORD_SIZE. With voice-steal banks, an overlapping release tail can add
     * up to CHORD_SIZE more — so worst-case peak is NUM_VOICES units. Divide
     * by CHORD_SIZE so the steady-state chord hits unit amplitude, then keep
     * 6 dB headroom by scaling to 16000 instead of 32767 (clipping path
     * catches the rare overlap-peak spikes). */
    const float voice_gain = 1.0f / (float)CHORD_SIZE;
    const float master_gain = inst->volume * 16000.0f;

    LFO *lfo = &inst->shape_lfo;

    for (int i = 0; i < frames; ++i) {
        /* Arp tick: if enabled and any notes are held, count down samples
         * to next step and advance pattern. */
        if (inst->arp_enabled && inst->held_count > 0) {
            inst->arp_sample_counter--;
            if (inst->arp_sample_counter <= 0) {
                inst->arp_sample_counter = inst->arp_step_period;
                /* Advance step_idx according to direction. */
                int hc = inst->held_count;
                switch (inst->arp_direction) {
                    case ARP_UP:
                        inst->arp_step_idx = (inst->arp_step_idx + 1) % hc;
                        break;
                    case ARP_DOWN:
                        inst->arp_step_idx--;
                        if (inst->arp_step_idx < 0) inst->arp_step_idx = hc - 1;
                        break;
                    case ARP_UPDOWN:
                        if (hc <= 1) {
                            inst->arp_step_idx = 0;
                        } else {
                            inst->arp_step_idx += inst->arp_step_dir;
                            if (inst->arp_step_idx >= hc) {
                                inst->arp_step_idx = hc - 2;
                                inst->arp_step_dir = -1;
                            } else if (inst->arp_step_idx < 0) {
                                inst->arp_step_idx = 1;
                                inst->arp_step_dir = 1;
                            }
                        }
                        break;
                    case ARP_RANDOM:
                        inst->arp_step_idx = rand() % hc;
                        break;
                }
                if (inst->arp_step_idx < 0) inst->arp_step_idx = 0;
                if (inst->arp_step_idx >= hc) inst->arp_step_idx = hc - 1;
                chord_on(inst, inst->held_notes[inst->arp_step_idx], 100);
            }
        }

        /* Advance shape LFO, compute effective shape offset for this sample. */
        lfo->phase += lfo->phase_inc;
        if (lfo->phase >= 1.0f) lfo->phase -= 1.0f;
        float shape_lfo_offset = lfo_sample(lfo->shape, lfo->phase) * inst->lfo_depth;

        /* Advance vibrato LFO + delay ramp, compute pitch-shift ratio. */
        inst->vibrato_phase += inst->vibrato_phase_inc;
        if (inst->vibrato_phase >= 1.0f) inst->vibrato_phase -= 1.0f;
        if (inst->vib_ramp_value < 1.0f) {
            inst->vib_ramp_value += inst->vib_ramp_inc;
            if (inst->vib_ramp_value > 1.0f) inst->vib_ramp_value = 1.0f;
        }
        /* Aftertouch additively boosts vibrato depth (clamped to 0..1). */
        float vib_depth_eff = inst->vib_depth + inst->aftertouch * 0.5f;
        if (vib_depth_eff > 1.0f) vib_depth_eff = 1.0f;

        float vib_mod_source;
        if (inst->vib_stray) {
            /* Smoothed random: pick a new random target every period_samples
             * and slew toward it. Period derived from vib_speed so it tracks
             * the user knob. */
            if (inst->vib_random_counter <= 0) {
                /* New target in [-1, +1]. */
                float r = ((float)(rand() & 0xFFFF) / 32768.0f) - 1.0f;
                inst->vib_random_target = r;
                int period = (int)(SAMPLE_RATE / (vib_speed_to_hz(inst->vib_speed) * 2.0f));
                if (period < 16) period = 16;
                inst->vib_random_counter = period;
            }
            inst->vib_random_counter--;
            /* Slew current toward target. */
            float slew = 0.002f;
            inst->vib_random_value += (inst->vib_random_target - inst->vib_random_value) * slew;
            vib_mod_source = inst->vib_random_value;
        } else {
            vib_mod_source = sinf(TWO_PI * inst->vibrato_phase);
        }

        float vib_cents = vib_mod_source
                          * inst->vib_ramp_value
                          * vib_depth_eff * VIB_DEPTH_MAX_CENTS;

        /* Pitch sweep: exp decay toward 0. */
        if (inst->sweep_value != 0.0f) {
            inst->sweep_value *= inst->sweep_coef;
            if (inst->sweep_value < 1e-4f && inst->sweep_value > -1e-4f) {
                inst->sweep_value = 0.0f;
            }
        }

        float pitch_semis = inst->sweep_value;
        float vib_ratio = (vib_cents == 0.0f && pitch_semis == 0.0f)
            ? 1.0f
            : exp2f(vib_cents * ONE_OVER_1200 + pitch_semis * ONE_OVER_12);

        float l_mix = 0.0f;
        float r_mix = 0.0f;

        for (int vi = 0; vi < NUM_VOICES; ++vi) {
            Voice *v = &inst->voices[vi];
            if (!v->active) continue;

            switch (v->env.stage) {
                case ENV_ATTACK:
                    v->env.value += v->env.attack_inc;
                    if (v->env.value >= 1.0f) {
                        v->env.value = 1.0f;
                        v->env.stage = ENV_HOLD;
                    }
                    break;
                case ENV_HOLD:
                    break;
                case ENV_RELEASE:
                    v->env.value *= v->env.release_coef;
                    if (v->env.value < ENV_SILENCE) {
                        v->env.value = 0.0f;
                        v->env.stage = ENV_IDLE;
                        v->active = false;
                    }
                    break;
                case ENV_IDLE:
                default:
                    v->active = false;
                    break;
            }

            if (!v->active) continue;

            /* Glide: ramp phase_inc toward target. */
            if (v->glide_step != 0.0f) {
                v->phase_inc += v->glide_step;
                if ((v->glide_step > 0.0f && v->phase_inc >= v->target_phase_inc) ||
                    (v->glide_step < 0.0f && v->phase_inc <= v->target_phase_inc)) {
                    v->phase_inc = v->target_phase_inc;
                    v->glide_step = 0.0f;
                }
            }

            float inc = v->phase_inc * vib_ratio;

            /* Per-osc shape + global shape LFO offset, clamped. */
            float eff_shape = inst->shapes[v->chord_step] + shape_lfo_offset;
            if (eff_shape < 0.0f) eff_shape = 0.0f;
            if (eff_shape > 1.0f) eff_shape = 1.0f;

            /* FM: carriers' phase lookup offset by last modulator sample. */
            float phase_used = v->phase;
            if (v->chord_step != inst->fm_modulator_idx && inst->fm_amount > 0.0f) {
                phase_used += inst->last_modulator_sample * inst->fm_amount;
                phase_used -= floorf(phase_used);
            }

            float s = osc_sample(v->waveform, phase_used, inc, eff_shape);
            v->phase += inc;
            if (v->phase >= 1.0f) v->phase -= 1.0f;
            else if (v->phase < 0.0f) v->phase += 1.0f;

            if (v->chord_step == inst->fm_modulator_idx) {
                inst->last_modulator_sample = s;
            }

            float amp = s * v->env.value * v->velocity * voice_gain
                          * inst->morph_gains[v->chord_step];

            /* Pan = width-based static pan + pan-morph offset, summed and
             * clamped. Equal-power L/R via sqrt. */
            float pan = 0.5f
                      + v->pan_offset * inst->width
                      + inst->pan_morph_pans[v->chord_step] * 0.5f;
            if (pan < 0.0f) pan = 0.0f;
            if (pan > 1.0f) pan = 1.0f;
            float pan_l = sqrtf(1.0f - pan);
            float pan_r = sqrtf(pan);

            l_mix += amp * pan_l;
            r_mix += amp * pan_r;
        }

        /* Filter coefficients: per-channel recompute when filter env or LFO is
         * active (the LFO supports L/R spread → independent coefs).
         * Otherwise reuse precomputed static coefs for both channels. */
        float al1 = inst->filter_a1, al2 = inst->filter_a2, al3 = inst->filter_a3;
        float ar1 = al1, ar2 = al2, ar3 = al3;
        bool env_on = inst->filter_env_depth != 0.0f;
        bool lfo_on = inst->filter_lfo_depth != 0.0f;

        if (env_on || lfo_on) {
            float env_val = env_on ? aenv_tick(&inst->filter_env) : 0.0f;
            float env_mod = env_val * inst->filter_env_depth;

            float lfo_l_mod = 0.0f, lfo_r_mod = 0.0f;
            if (lfo_on) {
                inst->filter_lfo_phase += inst->filter_lfo_phase_inc;
                if (inst->filter_lfo_phase >= 1.0f) inst->filter_lfo_phase -= 1.0f;
                float l_phase = inst->filter_lfo_phase;
                float r_phase = inst->filter_lfo_phase + inst->filter_lfo_spread * 0.5f;
                if (r_phase >= 1.0f) r_phase -= 1.0f;
                lfo_l_mod = lfo_sample(inst->filter_lfo_shape, l_phase) * inst->filter_lfo_depth;
                lfo_r_mod = lfo_sample(inst->filter_lfo_shape, r_phase) * inst->filter_lfo_depth;
            }

            float eff_l = inst->filter_cutoff + env_mod + lfo_l_mod;
            float eff_r = inst->filter_cutoff + env_mod + lfo_r_mod;
            if (eff_l < 0.0f) eff_l = 0.0f; if (eff_l > 1.0f) eff_l = 1.0f;
            if (eff_r < 0.0f) eff_r = 0.0f; if (eff_r > 1.0f) eff_r = 1.0f;

            float max_hz = SAMPLE_RATE * 0.49f;

            float hz_l = filter_cutoff_to_hz(eff_l);
            if (hz_l > max_hz) hz_l = max_hz; if (hz_l < 1.0f) hz_l = 1.0f;
            float g_l = tanf(3.14159265358979f * hz_l / SAMPLE_RATE);
            al1 = 1.0f / (1.0f + g_l * (g_l + inst->filter_k));
            al2 = g_l * al1;
            al3 = g_l * al2;

            float hz_r = filter_cutoff_to_hz(eff_r);
            if (hz_r > max_hz) hz_r = max_hz; if (hz_r < 1.0f) hz_r = 1.0f;
            float g_r = tanf(3.14159265358979f * hz_r / SAMPLE_RATE);
            ar1 = 1.0f / (1.0f + g_r * (g_r + inst->filter_k));
            ar2 = g_r * ar1;
            ar3 = g_r * ar2;
        }

        /* 12 dB: single SVF pass per channel. 24 dB: cascade two passes. */
        float fl = svf_process(&inst->filter_l, l_mix, inst->filter_mode,
                               al1, al2, al3, inst->filter_k);
        float fr = svf_process(&inst->filter_r, r_mix, inst->filter_mode,
                               ar1, ar2, ar3, inst->filter_k);
        if (inst->filter_slope == 1) {
            fl = svf_process(&inst->filter_l_b, fl, inst->filter_mode,
                             al1, al2, al3, inst->filter_k);
            fr = svf_process(&inst->filter_r_b, fr, inst->filter_mode,
                             ar1, ar2, ar3, inst->filter_k);
        }

        /* Resonance compensation — see notes earlier. */
        float reso_comp = 1.0f / (1.0f + inst->filter_resonance * 3.0f);
        fl *= reso_comp;
        fr *= reso_comp;

        /* Drive — soft-clip via tanh. */
        float drive_gain = 1.0f + inst->drive * 9.0f;
        float dl = tanhf(drive_gain * fl);
        float dr = tanhf(drive_gain * fr);

        /* Lo-Fi: bit grind (with optional DC shift bias), then sample-rate
         * decimation (sample/hold). Bypassed cleanly when params are 0. */
        if (inst->grind > 0.0f) {
            float bits = 16.0f - inst->grind * 14.0f;   /* 16 → 2 */
            float steps = powf(2.0f, bits) * 0.5f;      /* half because signal is ±1 */
            float shift = inst->bit_shift * 0.5f;       /* up to ±0.5 bias */
            float bl = dl + shift;
            float br = dr + shift;
            dl = (floorf(bl * steps + 0.5f) / steps) - shift;
            dr = (floorf(br * steps + 0.5f) / steps) - shift;
        }

        if (inst->decimator > 0.0f) {
            int hold = (int)(inst->decimator * 32.0f);
            if (hold < 1) hold = 1;
            if (inst->decim_counter <= 0) {
                inst->decim_hold_l = dl;
                inst->decim_hold_r = dr;
                inst->decim_counter = hold;
            }
            inst->decim_counter--;
            dl = inst->decim_hold_l;
            dr = inst->decim_hold_r;
        }

        /* Delay: runs after lo-fi, before reverb. Three modes:
         *  0 (stereo)    – independent L/R lines, feedback per channel
         *  1 (ping-pong) – cross-feedback (L←R, R←L), single tap input mono'd
         *  2 (flip-flop) – ping-pong + swap which channel is read on each
         *                  delay period for ear-pinging stereo motion. */
        if (inst->delay_mix > 0.0f || inst->delay_feedback > 0.0f) {
            float delay_s = DELAY_TIME_MIN_S +
                            inst->delay_time * (DELAY_TIME_MAX_S - DELAY_TIME_MIN_S);
            int delay_samples = (int)(delay_s * SAMPLE_RATE);
            if (delay_samples < 1) delay_samples = 1;
            if (delay_samples >= DELAY_BUFFER_SIZE) delay_samples = DELAY_BUFFER_SIZE - 1;

            int read_idx = (inst->delay_write_idx - delay_samples) & DELAY_BUFFER_MASK;
            float wet_l = inst->delay_buf_l[read_idx];
            float wet_r = inst->delay_buf_r[read_idx];

            float lp = 0.05f + inst->delay_tone * 0.95f;
            inst->delay_lp_l = lp * wet_l + (1.0f - lp) * inst->delay_lp_l;
            inst->delay_lp_r = lp * wet_r + (1.0f - lp) * inst->delay_lp_r;

            float fb = inst->delay_feedback * DELAY_FEEDBACK_MAX;
            float in_l = dl, in_r = dr;
            float out_l = wet_l, out_r = wet_r;

            if (inst->delay_mode == DELAY_STEREO) {
                inst->delay_buf_l[inst->delay_write_idx] = in_l + inst->delay_lp_l * fb;
                inst->delay_buf_r[inst->delay_write_idx] = in_r + inst->delay_lp_r * fb;
            } else {
                /* Ping-pong & flip-flop: cross-feedback L↔R. */
                float in_mono = (in_l + in_r) * 0.5f;
                inst->delay_buf_l[inst->delay_write_idx] = in_mono + inst->delay_lp_r * fb;
                inst->delay_buf_r[inst->delay_write_idx] = inst->delay_lp_l * fb;

                if (inst->delay_mode == DELAY_FLIPFLOP) {
                    /* Swap output read each delay period — counter advances
                     * when write_idx wraps past a period boundary. */
                    if ((inst->delay_write_idx % delay_samples) == 0) {
                        inst->delay_flip_counter++;
                    }
                    if (inst->delay_flip_counter & 1) {
                        float tmp = out_l; out_l = out_r; out_r = tmp;
                    }
                }
            }
            inst->delay_write_idx = (inst->delay_write_idx + 1) & DELAY_BUFFER_MASK;

            float mix = inst->delay_mix;
            dl = dl * (1.0f - mix) + out_l * mix;
            dr = dr * (1.0f - mix) + out_r * mix;
        }

        /* Reverb: 4 parallel combs summed + 2 series allpass per channel. */
        float wl = dl, wr = dr;
        if (inst->reverb_mix > 0.0f) {
            float rl =
                comb_process(&inst->rev_comb_l[0], dl) +
                comb_process(&inst->rev_comb_l[1], dl) +
                comb_process(&inst->rev_comb_l[2], dl) +
                comb_process(&inst->rev_comb_l[3], dl);
            float rr =
                comb_process(&inst->rev_comb_r[0], dr) +
                comb_process(&inst->rev_comb_r[1], dr) +
                comb_process(&inst->rev_comb_r[2], dr) +
                comb_process(&inst->rev_comb_r[3], dr);
            rl *= 0.25f;
            rr *= 0.25f;
            rl = allpass_process(&inst->rev_ap_l[1], allpass_process(&inst->rev_ap_l[0], rl));
            rr = allpass_process(&inst->rev_ap_r[1], allpass_process(&inst->rev_ap_r[0], rr));

            wl = dl * (1.0f - inst->reverb_mix) + rl * inst->reverb_mix;
            wr = dr * (1.0f - inst->reverb_mix) + rr * inst->reverb_mix;
        }

        float sl = wl * master_gain;
        float sr = wr * master_gain;
        if (sl > 32767.0f) sl = 32767.0f;
        if (sl < -32768.0f) sl = -32768.0f;
        if (sr > 32767.0f) sr = 32767.0f;
        if (sr < -32768.0f) sr = -32768.0f;

        out_interleaved_lr[2 * i + 0] = (int16_t)sl;
        out_interleaved_lr[2 * i + 1] = (int16_t)sr;
    }
}

/* ------------------------------------------------------------------------- */
/* v2 entry point                                                            */
/* ------------------------------------------------------------------------- */

static plugin_api_v2_t g_plugin_api_v2;

extern "C" plugin_api_v2_t* move_plugin_init_v2(const host_api_v1_t *host) {
    g_host = host;
    memset(&g_plugin_api_v2, 0, sizeof(g_plugin_api_v2));
    g_plugin_api_v2.api_version = MOVE_PLUGIN_API_VERSION_2;
    g_plugin_api_v2.create_instance = v2_create_instance;
    g_plugin_api_v2.destroy_instance = v2_destroy_instance;
    g_plugin_api_v2.on_midi = v2_on_midi;
    g_plugin_api_v2.set_param = v2_set_param;
    g_plugin_api_v2.get_param = v2_get_param;
    g_plugin_api_v2.get_error = v2_get_error;
    g_plugin_api_v2.render_block = v2_render_block;
    return &g_plugin_api_v2;
}
