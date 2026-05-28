# Chordism

A chord-based polyphonic synthesizer for Schwung. One note in, four voices out, with morphing level/pan and a stereo filter chain.

Loads as a `sound_generator` in Schwung's Signal Chain.

---

## Install

Download `chordism-module.tar.gz` from the [latest release](https://github.com/charlesvestal/schwung-chordism/releases) and extract on the device:

```
ssh ableton@move.local mkdir -p /data/UserData/schwung/modules/sound_generators/chordism
scp chordism-module.tar.gz ableton@move.local:/tmp/
ssh ableton@move.local "tar -xzf /tmp/chordism-module.tar.gz -C /data/UserData/schwung/modules/sound_generators/"
```

Or use `schwung-manager` pointed at this repo's releases.

Restart Schwung to pick up the module, then add it to a Signal Chain slot.

---

## Concept

Press one pad → hear a 4-voice chord. New pad takes over (mono input, polyphonic output). Release that pad and the previously held pad returns. Press harder for filter movement.

- **Last-note priority** with a held-note stack.
- **Polyphonic aftertouch** (Move pads) is hardcoded to open the filter cutoff.
- 30 presets cover the territory; each shows off a different angle.

---

## Signal Path

```
note → tuning (chord type or interval list, scale-quantized)
     → 4 oscillators (per-osc waveform + shape + per-osc LFO phase)
     → FM matrix (one mod, three carriers)
     → Level Morph mix (LUT × intensity + LFO)
     → Pan Morph (LUT × intensity + LFO) + width
     → stereo Mixer trims
     → Lo-Fi (pre-filter optional)
     → Filter (TPT SVF, LP/HP/BP × 12/24 dB, env + LFO)
     → Drive (tanh, bypassed when zero)
     → Lo-Fi (post-filter default)
     → Delay (6 modes)
     → Reverb (Schroeder)
     → output
```

---

## UI Navigation

Root level + sub-levels. Knobs map to physical encoders. Some params are accessible only via the menu list.

| Level | What's there |
|-------|--------------|
| **root** | Chord, Width, Cutoff, Reso, Drive, Shape, Reverb Mix, Volume + presets |
| **Oscillators** | Wave 1..4, Shape 1..4, Chord, Spread, Rotation, Detune, Width, Morph, FM |
| **Filter** | Cutoff, Reso, Mode (LP/HP/BP), Slope (12/24), Env A/D/Amt, Env Mode, Hard Reset, Filter LFO, Drive |
| **Modulation** | Shape LFO, Vibrato (depth/speed/delay/stray + per-osc enable), Pitch Sweep, Glide, Detune |
| **Envelope** | Attack, Release, Volume, VCA Mode, Hard Reset, Drone |
| **FX** | Reverb (mix/decay/damp/bokeh/lowcut/space/mod) + Lo-Fi (grind/shift/decim) + position toggle |
| **Delay** | Mix, Time, Feedback, Tone Hi/Lo, Mode (6), Mod |
| **Arp** | Enabled, Tempo, Direction, Hold, Euclidean, Clock Sync, Clock Division |
| **Morph** | Level Morph index/intensity/LFO + Pan Morph index/intensity/LFO |
| **Mixer** | Mix 1..4 (per-osc levels), FM Amt 1..4, FM Modulator, FM Position, Osc enables for vib/sweep |
| **Scale** | Scale (25 scales), Scale Root (C..B), Tuning Mode, Interval 1/2/3 |
| **Ctrl Src** | Source (Aftertouch/Random/Coin Toss/MIDI CC/Velocity), CC #, routings → Cutoff/Morph/Vib/Shape/FM |
| **Chord Multi** | Per-pitch-class chord_type (C, C#, D, ..., B) |

---

## Sound Architecture

### Tuning Modes

- **Chord** — chord_type LUT selects voicing. 16 chord types (Octaves, Fifth, Min, Min7..11, Maj, Maj7..9, Sus4, 6/9, Min6, 10th, Dom7, Dom7b9, Half-Dim).
- **Interval** — user-settable Interval 1/2/3 (-24..+24 semis each). Voice 0 is always root.
- **Chord Multi** — each pitch class (C, C#, D, ..., B) carries its own chord_type. Press C → Major, press A → Minor, etc. — same engine but the chord changes with what you play.

Spread scales all intervals (0 → unison, 0.5 → original, 1 → 2× wide). Rotation shifts which voice plays which chord degree.

### Scale Quantizer

25 scales: Chromatic (bypass), Major, Natural Minor, Harmonic Minor, Pentatonic Maj/Min, Diminished, Dorian, Phrygian, Lydian, Mixolydian, Locrian, Blues Maj/Min, Arabic (3 variants), Iwato, Pelog, Slendro, Folk, Japanese, Gypsy, Flamenco, Whole Tone. Chord notes snap to the selected scale degree.

### Waveforms

7 options per voice (`wave_1`..`wave_4`):

| Wave | Shape parameter behavior |
|------|--------------------------|
| Off | silence |
| Sine | Buchla-style wavefolder (shape = fold gain) |
| Triangle | Saw-tilt (peak position shifts toward 0 as shape rises) |
| Saw | Octave-morph (crossfade between fundamental and octave-up) |
| Square | PWM (5%..95%) |
| Pulse Train | Narrow PWM (2%..30%) — buzzy/thin character |
| Wavetable | Sum-of-harmonics, shape controls bandwidth (1..8 partials) |

Saw, Square, and Pulse Train use PolyBLEP for anti-aliasing.

### FM Matrix

Pick one chord voice as the modulator (`fm_modulator`, 0..3). Its output adds to the phase of the other three carriers, scaled by global `fm_amount` × per-carrier `fm_amount_1`..`fm_amount_4`. Position toggle — pre-morph keeps FM constant; post-morph means morph levels affect FM depth.

### Level Morph

16 hand-authored rows of 4-voice gain mixes (silent voice, ramp up, alternating, all-up, log up, etc.). `morph_index` sweeps with linear interp. `morph_intensity` blends flat (1,1,1,1) toward the row. `lm_lfo_*` animates the index over time.

### Pan Morph + Width

16 rows of 4-voice pan positions (-1..+1). `pan_morph_index` + `pan_morph_intensity` analogous to level morph. `width` independently scales the chord's static spread (voice 0 left → voice 3 right). The two combine.

### Per-osc Shape LFO Phase

The shape LFO is shared (one rate, depth, shape) but each voice uses its own phase offset (`lfo_phase_1`..`lfo_phase_4`, default staggered 0/0.25/0.5/0.75). Result: shape modulation moves through the chord rather than thumping all voices in sync.

### Filter

TPT SVF (topology-preserving — stable at any cutoff). Dual stage (one per channel). Modes LP/HP/BP. Slope 12 dB (single pass) or 24 dB (cascade). Resonance peak compensation: 4× post-attenuation at 12 dB, 7× at 24 dB.

- **Filter Envelope**: per-chord AD/ASR/Looping with hard-reset flag. Bipolar depth.
- **Filter LFO**: rate, depth, shape, L/R phase **spread** (offsets the right channel for auto-pan-like motion). Also has LFO modes (free / note-reset / one-shot).
- **Aftertouch** hardcoded to open cutoff by up to +0.4. Always on, every preset.

### Drive

`tanh(gain × x)` only when drive > 0. At drive = 0 the path is fully transparent — no soft-clip, the clean signal is unchanged.

### Lo-Fi (Quality Control)

- **Grind**: bit reduction (16 → 2 bits).
- **Shift**: DC bias added before quantization, removed after — varies staircase character.
- **Decimator**: sample-rate hold (1..32 samples).
- **Position**: pre-filter or post-filter (default post).

### Envelope Modes

VCA env + Filter env each have:
- **AD** — Attack then auto-Decay. Gate-independent.
- **ASR** — Attack → Hold (while gate) → Release. Legacy.
- **Looping** — Attack → Decay → Attack → ... while gate held; Release on gate off.
- **Hard Reset** — when on, env value snaps to 0 on retrigger. Off (default) = soft retrigger from current value.
- **Drone** (VCA only) — bypasses VCA env entirely. Voices stay open at peak.

### Reverb

Schroeder topology: 4 parallel combs + 2 series allpass per channel. Params: Mix, Decay (comb feedback), Damp (HF damping in feedback), Bokeh (allpass diffusion coefficient), Low Cut (one-pole HP on wet output), Space (comb-length scaler), Mod (subtle L/R out-of-phase damp modulation).

### Delay

6 modes:
- **Stereo** — independent L/R lines, feedback per channel.
- **Ping-Pong** — cross-feedback (L ← R, R ← L).
- **Flip-Flop** — ping-pong + L/R swap each delay period.
- **Long** — mono single line.
- **Zenith** — each repeat pitch-shifted +1 octave.
- **Interval** — each repeat pitch-shifted by `delay_mod_depth` (-12..+12 semis).

Tone Hi/Lo split, mod rate/depth on the delay time.

### Arpeggiator

- Directions: Up, Down, Up/Down, Random.
- **Hold** — latch held notes (note-off ignored while on).
- **Euclidean rhythm** — set steps (1..16) and beats; arp fires only on euclidean-distributed steps.
- **Variation** — interval ± 12 semis × count 1..8. Pattern transposes through variations over successive cycles.
- **Clock sync** — internal sample counter, or MIDI Clock (0xF8) with divisions 1/4, 1/4T, 1/8, 1/8T, 1/16, 1/32. Honors Start (0xFA) / Continue (0xFB) / Stop (0xFC).

### Modulation Source Matrix

One global control source routed to up to 5 targets with bipolar depth knobs.

| Source | Updates on |
|--------|------------|
| Aftertouch | continuous polyphonic pressure (also hardcoded to filter cutoff) |
| Random | new -1..+1 per note-on |
| Coin Toss | -1 or +1 per note-on |
| MIDI CC | the value of the configured CC (default CC 1 = mod wheel) |
| Velocity | 0..1 per note-on velocity |

Routings: `ctrl_to_cutoff`, `ctrl_to_morph`, `ctrl_to_vib`, `ctrl_to_shape`, `ctrl_to_fm`.

---

## Presets (30)

| # | Name | What to expect |
|---|------|----------------|
| 0 | Init | Plain sine major chord, soft AR. |
| 1 | Lush Pad | Wide saw pad, slow attack, level/pan morph, reverb. |
| 2 | FM Bell | Bell-like sine with FM. |
| 3 | Plucky Lead | Bright saw chord (10th), short envelope. |
| 4 | Filtered Sweep | Filter LFO + filter env sweeping. |
| 5 | Ambient Drone | Slow sine/triangle drone, big reverb. |
| 6 | Arp Bliss | Arp Up, saws, ~180 BPM. |
| 7 | Lo-Fi Stab | Saw chord, bit-crushed. |
| 8 | Glacial | Glacial saw pad with heavy reverb tail. |
| 9 | Acid Lead | 303-style Fifth, 24 dB filter, high reso, drive, glide-legato. |
| 10 | Sub Bass | Single sine (voice 0 only) on octaves. |
| 11 | EP | Sine with shape wavefold + mild FM. |
| 12 | Synth Brass | Saws with snappy filter-env open on attack. |
| 13 | Underwater | Pan morph + filter LFO spread + flip-flop delay. |
| 14 | Trance Arp | Arp Up/Down + 24 dB filter env + drive + ping-pong delay. |
| 15 | Phrygian Pad | Phrygian scale + animated morphs. |
| 16 | Pentatonic Pluck | Pent minor scale, short envelope. |
| 17 | Wavetable Drone | Wavetable wave + slow morph LFOs. |
| 18 | Pulse Stab | Pulse train + short stabby env. |
| 19 | Aftertouch Wow | Press pads harder → filter opens + vib boost + brighter shape. |
| 20 | Tape Echo | Flip-flop delay + delay mod + lo-fi. |
| 21 | Cosmic Sweep | Everything on — animated morphs + filter LFO spread + flip-flop. |
| 22 | Dub Bass | Sub + Long delay mode. |
| 23 | Random Bleeps | Random source + Euclidean arp + ping-pong. |
| 24 | Lo-Fi Pad | Pulse train + grind + animated morphs + heavy reverb. |
| 25 | Gypsy Lead | Gypsy scale + arp + drive. |
| 26 | Coin Toss Lead | Each note flips shape via coin-toss source. |
| 27 | Flamenco Stab | Flamenco scale on Dom7 + drive. |
| 28 | Looping Pulse | VCA Looping mode → pulse retriggers while held. |
| 29 | Wavetable Brass | Wavetable wave + filter env. |

---

## Tips

- The 4 chord voices use round-robin into 16 voice slots (4 banks of 4) so chord steals don't click — release tails finish before reuse.
- Filter env amount is bipolar. Negative = closes filter on attack.
- For glide to feel right, set Tuning = Chord and use a moderate Glide rate (~0.3). With **Glide Legato** on, glide only fires when notes overlap.
- Reverb Space at the top + Bokeh near max gives the biggest tail.
- For pure synthesis without saturation, leave Drive at 0 — the tanh is bypassed.
- Hold a pad through one MIDI clock period to verify clock-synced arp.
- The Move display only shows ~4 params at a time per page. Long press to scroll if a knob row has more than fits.

---

## Build

```
./scripts/build.sh         # Docker cross-compile to ARM64
./scripts/install.sh       # scp dist/chordism/ to Move
```

`build.sh` produces `dist/chordism-module.tar.gz`. Tarball contents:
- `module.json` — manifest with capabilities
- `help.json` — on-device manual (this file, in a format Move's Help viewer can render)
- `dsp.so` — the ARM64 plugin binary
- `LICENSE`

GitHub Actions builds and attaches the tarball to a release when you push a `v*` tag.

---

## License

MIT. See `LICENSE`.
