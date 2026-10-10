# Comb Voice v2 (Disting NT plug-in)

Stereo comb-filter synth voice **and** effect: two comb x mutation engines, chaos, freeze, a swirling in-loop
modulation section, and a Koszalin-style frequency shifter with regeneration.

Files to put in the GitHub repo: comb_voice.cpp, comb_core.h, nt_math.h, Makefile, .github/workflows/build.yml.

## Using it as an effect (external signals only)
- **Osc level** (Input/Osc page) = 0 silences the internal noise oscillator.
- Patch audio to **Audio in** (and **Audio in R** for stereo). **Input level** (0-200%) drives it into the combs.
- **Dry level** passes the untouched input to the outputs; **Wet level** sets the comb output.
- Pitch CV (1 V/oct) still tunes the combs.
- Frequency shifter only: Osc level 0, Dry 100, Wet 0, Shifter = Output.

## Frequency shifter (Shifter page)
- **Shifter**: Off / Input (before the combs) / Loop (inside the feedback loops: each pass shifts again, so repeats
  spiral up or down) / Output.
- **Shift mode**: Up, Down, Split (L up, R down), Ring (both sidebands).
- **Shift range** x1/x10/x100, **Shift** (+/-50 Hz in 0.1 steps x range) and **Shift fine**. Negative values go through zero.
- **Lin FM in / depth**: through-zero linear FM, 100% = 100 Hz per volt (x range).
- **Exp FM in / depth**: exponential FM, 100% = 1 octave per volt applied to the shift amount.
- **Shifter mix**: shifted vs unshifted.
- **Regen page**: feedback around the shifter (Input/Output positions) with a short delay, routed from the Down, Up
  or Both sidebands; the Koszalin-style barberpole/flanger-ish spirals.

## Swirl (Swirl/Mod page)
- **Swirl** (default 50%) adds slow pitch wobble per loop (up to +/-45 cents), swept allpass chains inside the loops and
  rotation of the feedback between left and right. **Swirl rate** sets speed (0.03 - 3.8 Hz). 0 = old sound.

## Notes
- Existing parameters keep their old numbers; new ones are appended. Input level now defaults to 100%.
- Tested on a host PC with a stand-in API header only; the ARM build runs in GitHub Actions.
