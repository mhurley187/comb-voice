# Comb Voice - disting NT plug-in

A stereo synth voice built around comb filters, in the spirit of the Isobar Serratus (comb x mutation
matrix, chaos, freeze) and Vhikk X (detuned stereo drone voice). It is a **plug-in**: C++ compiled to an
ARM `.o` file that you copy to the disting NT's SD card.

## Signal flow
Exciter (noise, or noise bursts on a gate, plus optional external audio) -> two comb **engines**
(A and B), each a left and right delay loop, detuned against each other. Inside each loop:
delay -> comb-specific filtering -> damping -> feedback gain -> **mutation** -> DC block -> soft limit.
**Morph** crossfades A -> B. Output: width, level, soft clip.

## Combs (9 in the Serratus; 6 here)
| Comb | What it does |
|---|---|
| Classic | feedback comb, all harmonics |
| Hollow | negative feedback: odd harmonics only, clarinet/square-ish |
| String | Karplus-Strong style 2-point average loop |
| Metal | allpass chain in the loop: stiff, inharmonic, bell/metal (Dispersion sets the amount) |
| Thick | two taps +/-6 cents apart: phasey, beating resonance |
| Cross | left and right loops feed each other: stereo coupling |

## Mutations (sit inside the feedback loop; amount = "Mutation amt")
Clean, Saturate, Fold, Crush (bit depth), Clip (asymmetric), Decimate (sample-rate reduction), Rectify.

## Parameters
- **Tone**: Coarse (semitones), Fine (cents), Spread (L/R detune), Feedback, Damping, Dispersion.
  Feedback is a *decay time* (about 5 ms to 17 s, the same at every pitch); **98%+ self-oscillates**.
- **Engines**: Comb A/B, Mutation A/B, Morph A>B, Mutation amt.
- **Exciter**: Drone (continuous noise) or Pluck (burst on gate rising edge, Pluck decay), Exciter level,
  Input level (external audio into the loops), **Freeze** (stops input, holds the loop at unity gain).
- **Mod/Output**: Chaos (slow random drift of pitch, decay and damping, independent per side), Chaos rate,
  Width, Level.
- **Routing**: Pitch CV (1 V/oct, 0 V = C4), Gate, Audio in, Left/Right outputs with Add/Replace.

## Build and install
1. `git clone https://github.com/expertsleepersltd/distingNT_API` and install `arm-none-eabi-g++`.
2. `make API=/path/to/distingNT_API/include`  ->  `plugins/comb_voice.o`
3. Copy the `.o` into the plug-ins folder on the NT's SD card (see the NT manual for the exact path),
   power-cycle, and add "Comb Voice" from the algorithm list.
4. Before sharing, change the 4-character GUID in `comb_voice.cpp` to something unique.

## What has and has not been verified
Verified on a desktop CPU (`make test`): pitch accuracy, stability and boundedness across every comb x mutation
x extreme setting (including 100% feedback, freeze, max chaos, +/-5 V pitch), drone level consistency, decay
time vs the Feedback knob, and the plug-in's parameter/bus wiring against a **stand-in** API header.

**Not verified** (no ARM toolchain or real API headers were available when this was written):
- compiling against the real distingNT API - names such as `kNT_unitCents` or the `_NT_factory` fields may
  differ slightly in your API version; the compiler will point at them
- whether bus values are volts (assumed: audio scaled by 5, pitch CV used directly; see `kAudioVolts`)
- CPU load on the NT (about 1.4% of one desktop core; check the NT's CPU meter) and whether the NT's
  loader resolves `sinf/expf/atan2f/powf` for plug-ins
- how it sounds on real material: all tuning above is by measurement, not by ear
