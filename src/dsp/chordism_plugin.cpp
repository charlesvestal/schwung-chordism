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
static const int   NUM_VOICES = 8;   /* 2 banks of CHORD_SIZE — old chord can ring out while new chord plays */

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

enum Waveform { WAVE_SINE = 0, WAVE_TRIANGLE = 1, WAVE_SAW = 2, WAVE_SQUARE = 3 };
static const int NUM_WAVEFORMS = 4;

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
    float phase;
    float phase_inc;
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
    int   waveform;      /* 0..NUM_WAVEFORMS-1 */
    float shape;         /* 0..1 — per-waveform variable attribute */
    float lfo_rate;      /* 0..1 — exp-mapped to Hz */
    float lfo_depth;     /* 0..1 — modulation amount on shape */
    int   lfo_shape;     /* 0..NUM_LFO_SHAPES-1 */
    int   chord_type;    /* 0..NUM_CHORDS-1 — row in CHORD_TABLE */
    float detune;        /* 0..1 → 0..MAX_DETUNE_CENTS per chord step */
    float width;         /* 0..1 — 0=mono, 1=full LR spread across chord */
    float filter_cutoff;     /* 0..1 → exp 20..20kHz */
    float filter_resonance;  /* 0..1 → Q from FILTER_Q_MIN to FILTER_Q_MAX */
    int   filter_mode;       /* 0..NUM_FILTER_MODES-1 */
    float filter_env_attack; /* 0..1 → linear A time */
    float filter_env_decay;  /* 0..1 → exp D time */
    float filter_env_depth;  /* -1..+1 — bipolar modulation of cutoff */
    float drive;             /* 0..1 → 1..10x pre-tanh gain */

    SVF   filter_l;
    SVF   filter_r;
    /* Shared TPT SVF coefficients (same cutoff/Q across L and R), precomputed
     * at base cutoff (filter_recompute). If filter_env_depth != 0,
     * render_block recomputes per-sample. */
    float filter_a1;
    float filter_a2;
    float filter_a3;
    float filter_k;

    ADEnv filter_env;

    LFO   shape_lfo;
    Voice voices[NUM_VOICES];
    int next_chord_base;   /* 0 or CHORD_SIZE — round-robin between voice banks */
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
    v->phase = 0.0f;
    float hz = midi_to_hz(root_note + interval_semis);
    if (detune_cents != 0.0f) {
        hz *= powf(2.0f, detune_cents / 1200.0f);
    }
    v->phase_inc = hz / SAMPLE_RATE;
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

    for (int i = 0; i < CHORD_SIZE; ++i) {
        Voice *target = &inst->voices[base + i];
        /* Detune linear: voice 0 = 0¢, voice 1 = +detune*MAX¢, ... */
        float cents = (float)i * inst->detune * MAX_DETUNE_CENTS;
        voice_start(inst, target, root_note, intervals[i], cents, velocity, i);
    }
    inst->next_chord_base = (base + CHORD_SIZE) % NUM_VOICES;

    /* Re-trigger filter envelope from the start (hard reset to 0). */
    aenv_recompute_rates(&inst->filter_env,
                         inst->filter_env_attack, inst->filter_env_decay);
    inst->filter_env.value = 0.0f;
    inst->filter_env.stage = ENV_ATTACK;
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
    inst->waveform = WAVE_SINE;
    inst->shape = 0.0f;
    inst->lfo_rate = 0.0f;
    inst->lfo_depth = 0.0f;
    inst->lfo_shape = LFO_TRIANGLE;
    inst->chord_type = CHORD_MAJOR;
    inst->detune = 0.0f;
    inst->width = 1.0f;               /* full stereo spread by default */
    inst->filter_cutoff = 1.0f;       /* wide open by default */
    inst->filter_resonance = 0.0f;    /* no resonance */
    inst->filter_mode = FILT_LP;
    inst->filter_env_attack = 0.0f;
    inst->filter_env_decay = 0.30f;
    inst->filter_env_depth = 0.0f;    /* disabled by default */
    inst->filter_env.stage = ENV_IDLE;
    inst->filter_env.value = 0.0f;
    aenv_recompute_rates(&inst->filter_env,
                         inst->filter_env_attack, inst->filter_env_decay);
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
        chord_on(inst, d1, d2);
    } else if (status == 0x80 || (status == 0x90 && d2 == 0)) {
        chord_off(inst, d1);
    } else if (status == 0xB0 && d1 == 123) {
        release_all_voices(inst);
    }
}

static void v2_set_param(void *instance, const char *key, const char *val) {
    if (!instance || !key) return;
    auto *inst = (chordism_instance_t*)instance;

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
        if (val) {
            int w = atoi(val);
            if (w < 0) w = 0;
            if (w >= NUM_WAVEFORMS) w = NUM_WAVEFORMS - 1;
            inst->waveform = w;
        }
    } else if (strcmp(key, "shape") == 0) {
        inst->shape = param_from_string(val, inst->shape);
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
    }
}

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len) {
    if (!instance || !buf || buf_len <= 0) return 0;
    auto *inst = (chordism_instance_t*)instance;

    if (key && strcmp(key, "attack") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->attack);
    } else if (key && strcmp(key, "release") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->release);
    } else if (key && strcmp(key, "volume") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->volume);
    } else if (key && strcmp(key, "waveform") == 0) {
        return snprintf(buf, buf_len, "%d", inst->waveform);
    } else if (key && strcmp(key, "shape") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->shape);
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
    } else if (key && strcmp(key, "filter_cutoff") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_cutoff);
    } else if (key && strcmp(key, "filter_resonance") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_resonance);
    } else if (key && strcmp(key, "drive") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->drive);
    } else if (key && strcmp(key, "filter_mode") == 0) {
        return snprintf(buf, buf_len, "%d", inst->filter_mode);
    } else if (key && strcmp(key, "filter_env_attack") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_env_attack);
    } else if (key && strcmp(key, "filter_env_decay") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_env_decay);
    } else if (key && strcmp(key, "filter_env_depth") == 0) {
        return snprintf(buf, buf_len, "%.4f", inst->filter_env_depth);
    } else if (key && strcmp(key, "version") == 0) {
        return snprintf(buf, buf_len, "0.0.15");
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
        /* Advance shape LFO, compute effective shape value for this sample. */
        lfo->phase += lfo->phase_inc;
        if (lfo->phase >= 1.0f) lfo->phase -= 1.0f;
        float lfo_val = lfo_sample(lfo->shape, lfo->phase) * inst->lfo_depth;
        float effective_shape = inst->shape + lfo_val;
        if (effective_shape < 0.0f) effective_shape = 0.0f;
        if (effective_shape > 1.0f) effective_shape = 1.0f;

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

            float s = osc_sample(inst->waveform, v->phase, v->phase_inc, effective_shape);
            v->phase += v->phase_inc;
            if (v->phase >= 1.0f) v->phase -= 1.0f;

            float amp = s * v->env.value * v->velocity * voice_gain;

            /* Realtime equal-power pan from live width knob + voice's fixed
             * chord position. sqrt-based: L^2 + R^2 = 1. */
            float pan = 0.5f + v->pan_offset * inst->width;
            if (pan < 0.0f) pan = 0.0f;
            if (pan > 1.0f) pan = 1.0f;
            float pan_l = sqrtf(1.0f - pan);
            float pan_r = sqrtf(pan);

            l_mix += amp * pan_l;
            r_mix += amp * pan_r;
        }

        /* Filter coefficients: use precomputed static values, unless filter
         * envelope is active — then recompute per sample. Shared across L+R. */
        float a1 = inst->filter_a1;
        float a2 = inst->filter_a2;
        float a3 = inst->filter_a3;
        if (inst->filter_env_depth != 0.0f) {
            float env_val = aenv_tick(&inst->filter_env);
            float effective = inst->filter_cutoff + env_val * inst->filter_env_depth;
            if (effective < 0.0f) effective = 0.0f;
            if (effective > 1.0f) effective = 1.0f;

            float hz = filter_cutoff_to_hz(effective);
            float max_hz = SAMPLE_RATE * 0.49f;
            if (hz > max_hz) hz = max_hz;
            if (hz < 1.0f) hz = 1.0f;

            float g = tanf(3.14159265358979f * hz / SAMPLE_RATE);
            a1 = 1.0f / (1.0f + g * (g + inst->filter_k));
            a2 = g * a1;
            a3 = g * a2;
        }

        /* Dual SVF — same coefficients, independent state per channel. */
        float fl = svf_process(&inst->filter_l, l_mix, inst->filter_mode,
                               a1, a2, a3, inst->filter_k);
        float fr = svf_process(&inst->filter_r, r_mix, inst->filter_mode,
                               a1, a2, a3, inst->filter_k);

        /* Resonance compensation — see notes earlier. */
        float reso_comp = 1.0f / (1.0f + inst->filter_resonance * 3.0f);
        fl *= reso_comp;
        fr *= reso_comp;

        /* Drive — soft-clip via tanh. */
        float drive_gain = 1.0f + inst->drive * 9.0f;
        float dl = tanhf(drive_gain * fl);
        float dr = tanhf(drive_gain * fr);

        float sl = dl * master_gain;
        float sr = dr * master_gain;
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
