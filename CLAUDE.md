# Schwung Chordism

External Schwung module. Chord-based polyphonic synth, sound_generator chain slot.

## Build / Deploy

```
./scripts/build.sh         # Cross-compile via Docker
./scripts/install.sh       # scp dist/chordism/ to Move
```

After install, restart Schwung service on Move to pick up the new module.

## Architecture

- `src/module.json` — manifest, ui_hierarchy, capabilities
- `src/dsp/chordism_plugin.cpp` — plugin_api_v2 entry
- `scripts/{build.sh, Dockerfile, install.sh}` — build & deploy
- `.github/workflows/release.yml` — auto-build on `v*` tag

Polyphony: 4 voices, hard mono input, last-note priority.

## Status

Milestone 1: skeleton + silent plugin. See `../schwung/docs/plans/2026-05-27-chordism-design.md` for full design.

## License

MIT.
