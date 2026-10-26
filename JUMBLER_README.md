# Jumbler (disting NT)

Six inputs, six outputs, DC-coupled (audio or CV). Signals are crossfaded smoothly between neighbouring channels as you rotate.

- **Rotate in / Rotate out** (0-6 channels): rotate which input feeds which output. CV: 10 V = one full turn (6 channels), wraps seamlessly.
  Both are applied to the same circular routing, so their amounts add; the difference is that each has its own CV, Smooth/Stepped motion and Spin.
- **Radiate** (0-100%): each output blends in neighbouring channels (up to 3 away). Overall gain stays constant. CV: 10 V = full.
- **In motion / Out motion**: Smooth, or Stepped (snaps to whole channels, crossfaded over Slew).
- **Slew** (1-500 ms): how fast the routing glides after a change.
- **Spin in / Spin out** (+/-5 channels per second): free-running automatic rotation.
- Unpatched inputs are silent. Outputs default to buses 13-18; **Output mode** is Add/Replace for all six.
