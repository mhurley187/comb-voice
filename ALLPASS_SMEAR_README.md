# Allpass Smear (disting NT)

Stereo allpass filter bank: 2 to 98 poles (1 to 49 biquad sections per side), swept smoothly.
An allpass is flat on its own, so set **Mix** below 100% (default 50%) to hear notches where the wet and dry cancel.

**Filter page:** Poles (2.0-98.0, fractional - the last section fades in, so sweeping is click-free), Frequency (1 V/oct, 0 V = 20 Hz),
Resonance (+/- feedback; negative inverts the comb pattern), Spread (octaves the stages cover; 0 = all stages stacked at one frequency),
Peak bias (-100% crowds the notches toward the low end, +100% toward the high end), Sharpness (phase slope of each stage).
**Mod page:** LFO rate/depth/phase (R side offset), Env amount (input envelope to frequency, +/-), Stereo (L/R frequency split).
**Output:** Mix, Level. **Routing:** audio in L/R (R normalled to L), CV inputs for Poles, Frequency, Resonance, Spread, Bias, Sharpness, Mix, outputs with Add/Replace.

CV scaling: Frequency 1 V/oct; Poles 10 V = 96 poles; Resonance +/-5 V = full; Spread 10 V = 6 oct; Bias +/-5 V = full; Sharpness 10 V = full; Mix 10 V = full.
