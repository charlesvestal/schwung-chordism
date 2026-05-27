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

/* Placeholder chord — root + M3 + P5 + octave. The full chord LUT comes in
 * milestone 4. */
static const int CHORD_INTERVALS_SEMITONES[CHORD_SIZE] = { 0, 4, 7, 12 };

/* Attack range: 1 ms .. 4 s, linear ramp.
 * Release range: 5 ms .. 4 s, exponential decay (asymptotic). */
static const float ATTACK_MIN_S = 0.001f;
static const float ATTACK_MAX_S = 4.0f;
static const float RELEASE_MIN_S = 0.005f;
static const float RELEASE_MAX_S = 4.0f;

/* Silence threshold below which the envelope clamps to 0 and the voice idles. */
static const float ENV_SILENCE = 1e-4f;

enum EnvStage { ENV_IDLE, ENV_ATTACK, ENV_HOLD, ENV_RELEASE };

enum Waveform { WAVE_SINE = 0, WAVE_TRIANGLE = 1, WAVE_SAW = 2, WAVE_SQUARE = 3 };
static const int NUM_WAVEFORMS = 4;

struct AREnv {
    EnvStage stage;
    float value;
    float attack_inc;
    float release_coef;
};

struct Voice {
    bool active;
    int root_note;       /* MIDI root that spawned this voice (for note-off match) */
    float phase;
    float phase_inc;
    float velocity;
    AREnv env;
};

struct chordism_instance_t {
    char module_dir[512];

    /* Params */
    float attack;        /* 0..1 */
    float release;       /* 0..1 */
    float volume;        /* 0..1 */
    int   waveform;      /* 0..NUM_WAVEFORMS-1 */

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

static float osc_sample(int waveform, float phase, float phase_inc) {
    switch (waveform) {
        case WAVE_SINE:
            return sinf(TWO_PI * phase);

        case WAVE_TRIANGLE:
            /* Triangle's harmonics roll off as 1/n^2 — aliasing is
             * inaudible at audio rates. Compute directly. */
            return 4.0f * fabsf(phase - 0.5f) - 1.0f;

        case WAVE_SAW: {
            float s = 2.0f * phase - 1.0f;
            s -= poly_blep(phase, phase_inc);
            return s;
        }

        case WAVE_SQUARE: {
            /* Two discontinuities per cycle (phase=0 rising, phase=0.5
             * falling). PolyBLEP at each with opposite signs. */
            float t2 = phase + 0.5f;
            if (t2 >= 1.0f) t2 -= 1.0f;
            float s = (phase < 0.5f) ? 1.0f : -1.0f;
            s += poly_blep(phase, phase_inc);
            s -= poly_blep(t2, phase_inc);
            return s;
        }

        default:
            return 0.0f;
    }
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
                        int root_note, int interval_semis, int velocity) {
    v->active = true;
    v->root_note = root_note;
    v->phase = 0.0f;
    v->phase_inc = midi_to_hz(root_note + interval_semis) / SAMPLE_RATE;
    v->velocity = (float)velocity / 127.0f;

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
    for (int i = 0; i < CHORD_SIZE; ++i) {
        Voice *target = &inst->voices[base + i];
        voice_start(inst, target, root_note,
                    CHORD_INTERVALS_SEMITONES[i], velocity);
    }
    inst->next_chord_base = (base + CHORD_SIZE) % NUM_VOICES;
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
    } else if (key && strcmp(key, "version") == 0) {
        return snprintf(buf, buf_len, "0.0.4");
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

    for (int i = 0; i < frames; ++i) {
        float mix = 0.0f;

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

            float s = osc_sample(inst->waveform, v->phase, v->phase_inc);
            v->phase += v->phase_inc;
            if (v->phase >= 1.0f) v->phase -= 1.0f;

            mix += s * v->env.value * v->velocity * voice_gain;
        }

        float scaled = mix * master_gain;
        if (scaled > 32767.0f) scaled = 32767.0f;
        if (scaled < -32768.0f) scaled = -32768.0f;

        int16_t out = (int16_t)scaled;
        out_interleaved_lr[2 * i + 0] = out;
        out_interleaved_lr[2 * i + 1] = out;
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
