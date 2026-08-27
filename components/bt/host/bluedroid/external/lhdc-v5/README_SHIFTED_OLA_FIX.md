# LHDC V5 low-frequency leakage fix attempt: shifted low-overlap OLA

This patch changes the time-domain overlap-add for low-overlap HR windows.

Previous code treated the IMDCT output as a conventional full 50% overlap block:

```c
for (n = 0; n < N; n++) x[n] *= window[n];
for (n = 0; n < H; n++) pcm[n] = x[n] + overlap_buf[n];
for (n = 0; n < H; n++) overlap_buf[n] = x[H+n];
```

The reference LHDC V5 time module uses:

```c
num1 = N / 2;
num2 = N / 4;
num4 = num2 - repeating / 2;
```

and stores only `repeating` samples of overlap. That means the zero-padded front
part of the IMDCT block is not supposed to be emitted. For 96 kHz / 24-bit:

```c
N = 960;
H = 480;
repeating = 192;
num4 = 144;
```

The patched output is:

```c
for (n = 0; n < repeating; n++)
    pcm[n] = x[num4+n] * window[num4+n] + previous_tail[n];

for (n = repeating; n < H; n++)
    pcm[n] = x[num4+n];

for (n = 0; n < repeating; n++)
    previous_tail[n] = x[num4+H+n] * window[num4+H+n];
```

This should be tested on the 50 Hz case first. If it makes the muffle move or
improves sharply, the artifact was the OLA time alignment. If it does not change
at all, the artifact is upstream of IMDCT/OLA, likely entropy bit-plane alignment
or SNS inverse shaping.

Compile-time A/B:

```c
#define LHDC_USE_LEGACY_FULL_OLA
```

will restore the old full-window OLA path.
