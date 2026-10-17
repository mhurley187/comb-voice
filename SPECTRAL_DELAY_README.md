# Spectral Delay (disting NT plug-in)

Stereo multitap delay with real-time spectral resynthesis in the feedback loop.

**Signal flow:** input -> delay line -> up to 8 taps -> (tap mix) -> spectral analysis -> sine-oscillator resynthesis -> feedback (+ output).

## Delay page
- **Time**: total span of the tap pattern, 20 ms - 2.5 s (exponential; 50% = 250 ms). **Time CV**: 1 V/oct (each volt doubles it).
- **Taps** 1-8, **Tap slope** (level fall-off over the taps), **Tap pan** (taps alternate left / right).
- **Grouping**: -100% bunches the taps toward the END of the span, +100% toward the START, 0 spreads them evenly.
  **Grouping CV**: 5 V = +100%, so you can sweep the cluster in real time.
- **Feedback** up to 110% (soft-saturated so it cannot run away), **Damping** low-pass in the loop.

## Spectral page
The tap mix is analysed (FFT), the strongest partials are tracked, and a bank of sine oscillators re-synthesises them.
- **Resynth FB** / **Resynth out**: how much of the feedback signal / of the output is the resynthesis instead of the plain taps.
- **Shift**: transposes every partial (+/-24 st) - each pass through the loop shifts again. **Shift CV**: 1 V = 12 st.
- **Partials** 1-32 (how many peaks are tracked), **Smear** (blur / sustain across analysis frames), **Stretch** (inharmonic stretch),
  **Scatter** (random detune per partial), **Tilt** (+/-6 dB/oct around 1 kHz), **Width** (stereo spread of the partials).
- **Freeze** (or the **Freeze gate** input): holds the current spectrum as a sustained drone.
- **Quality**: Fine = 2048-point FFT (better for low notes, ~45 ms of latency inside the loop), Fast = 1024-point (~23 ms).

## Output page
**Mix** (dry/wet) and **Level**.

Memory: about 1.1 MB of DRAM.
