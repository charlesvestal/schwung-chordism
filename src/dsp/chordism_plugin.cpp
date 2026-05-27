/*
 * Chordism DSP Plugin for Schwung (Move Anything)
 *
 * MIT License. See LICENSE.
 *
 * Milestone 2: single sine voice + AR envelope, mono last-note priority.
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

/* Attack range: 1 ms .. 4 s, linear ramp.
 * Release range: 5 ms .. 4 s, exponential decay (asymptotic). */
static const float ATTACK_MIN_S = 0.001f;
static const float ATTACK_MAX_S = 4.0f;
static const float RELEASE_MIN_S = 0.005f;
static const float RELEASE_MAX_S = 4.0f;

/* Silence threshold below which the envelope clamps to 0 and the voice idles. */
static const float ENV_SILENCE = 1e-4f;

enum EnvStage { ENV_IDLE, ENV_ATTACK, ENV_HOLD, ENV_RELEASE };

struct AREnv {
    EnvStage stage;
    float value;
    float attack_inc;    /* linear increment per sample while in ATTACK */
    float release_coef;  /* exponential decay coefficient per sample while in RELEASE */
};

struct Voice {
    bool active;        /* true between note-on and envelope finishing */
    int note;           /* MIDI note number, 0..127 */
    float phase;        /* 0..1 phase accumulator */
    float phase_inc;    /* frequency / SAMPLE_RATE */
    float velocity;     /* 0..1 */
    AREnv env;
};

struct chordism_instance_t {
    char module_dir[512];

    /* Params (all normalized 0..1) */
    float attack;
    float release;
    float volume;

    Voice voice;
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

/* Linear map [0,1] -> [lo, hi]. */
static float lerp01(float t, float lo, float hi) {
    return lo + (hi - lo) * t;
}

static void env_recompute_rates(AREnv *env, float attack01, float release01) {
    float attack_s = lerp01(attack01, ATTACK_MIN_S, ATTACK_MAX_S);
    float release_s = lerp01(release01, RELEASE_MIN_S, RELEASE_MAX_S);

    float attack_samples = attack_s * SAMPLE_RATE;
    if (attack_samples < 1.0f) attack_samples = 1.0f;
    env->attack_inc = 1.0f / attack_samples;

    /* Exponential decay coefficient. After release_s seconds the envelope
     * reaches e^-5 ≈ 0.0067 of its starting value, which we then clamp to 0
     * via ENV_SILENCE. coef = exp(-5 / (release_s * SAMPLE_RATE)). */
    float release_samples = release_s * SAMPLE_RATE;
    if (release_samples < 1.0f) release_samples = 1.0f;
    env->release_coef = expf(-5.0f / release_samples);
}

static void voice_note_on(chordism_instance_t *inst, int note, int velocity) {
    Voice *v = &inst->voice;
    v->active = true;
    v->note = note;
    v->phase = 0.0f;
    v->phase_inc = midi_to_hz(note) / SAMPLE_RATE;
    v->velocity = (float)velocity / 127.0f;

    env_recompute_rates(&v->env, inst->attack, inst->release);
    v->env.stage = ENV_ATTACK;
    /* Soft retrigger: keep current env.value so we don't click; attack ramps
     * up from wherever we are. */
}

static void voice_note_off(chordism_instance_t *inst, int note) {
    Voice *v = &inst->voice;
    if (!v->active || v->note != note) return;
    v->env.stage = ENV_RELEASE;
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
    inst->voice.env.stage = ENV_IDLE;
    env_recompute_rates(&inst->voice.env, inst->attack, inst->release);

    if (g_host && g_host->log) {
        g_host->log("[chordism] instance created");
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
        voice_note_on(inst, d1, d2);
    } else if (status == 0x80 || (status == 0x90 && d2 == 0)) {
        voice_note_off(inst, d1);
    } else if (status == 0xB0 && d1 == 123) {
        /* All Notes Off */
        if (inst->voice.active) inst->voice.env.stage = ENV_RELEASE;
    }
}

static void v2_set_param(void *instance, const char *key, const char *val) {
    if (!instance || !key) return;
    auto *inst = (chordism_instance_t*)instance;

    if (strcmp(key, "attack") == 0) {
        inst->attack = param_from_string(val, inst->attack);
        env_recompute_rates(&inst->voice.env, inst->attack, inst->release);
    } else if (strcmp(key, "release") == 0) {
        inst->release = param_from_string(val, inst->release);
        env_recompute_rates(&inst->voice.env, inst->attack, inst->release);
    } else if (strcmp(key, "volume") == 0) {
        inst->volume = param_from_string(val, inst->volume);
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
    } else if (key && strcmp(key, "version") == 0) {
        return snprintf(buf, buf_len, "0.0.2");
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
    Voice *v = &inst->voice;

    /* Voice gain — convert 0..1 volume to int16 headroom. Keep 6 dB headroom
     * by scaling to 16000 instead of 32767 (so future polyphony has room). */
    const float gain = inst->volume * 16000.0f;

    for (int i = 0; i < frames; ++i) {
        float sample = 0.0f;

        if (v->active) {
            /* Envelope */
            switch (v->env.stage) {
                case ENV_ATTACK:
                    v->env.value += v->env.attack_inc;
                    if (v->env.value >= 1.0f) {
                        v->env.value = 1.0f;
                        v->env.stage = ENV_HOLD;
                    }
                    break;
                case ENV_HOLD:
                    /* gate held — value stays at 1 */
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

            /* Sine oscillator */
            float s = sinf(TWO_PI * v->phase);
            v->phase += v->phase_inc;
            if (v->phase >= 1.0f) v->phase -= 1.0f;

            sample = s * v->env.value * v->velocity;
        }

        float scaled = sample * gain;
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
