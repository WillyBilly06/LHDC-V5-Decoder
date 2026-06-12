# LHDC low-frequency diagnostic build: one firmware, runtime switching

This package keeps the shifted low-overlap OLA fix enabled by default, but the
remaining diagnostics are now runtime variables instead of compile-time #defines.
That means you can flash once, then switch modes from your app wrapper, UART
console, BLE debug command, menu, etc.

## Files added

- `lhdc_diag_config.h`
- `lhdc_diag_config.c`

Add `lhdc_diag_config.c` to your build sources.

## Default mode

The default is the current best-sounding mode:

```c
lhdc_diag_set_modes(0, 0, 0, 0);
```

Meaning:

- `force_ref_imdct_960 = 0`: use fast 960 IMDCT
- `sns_mode = 0`: current SNS divide path
- `disable_coeff_rev = 0`: keep coeff reversal enabled
- `ola_mode = 0`: shifted low-overlap OLA

## Runtime tests, no reflashing

Call these one at a time while playing the same 50 Hz / 96 kHz / 24-bit stream.
Switch back to baseline before trying the next test.

### Baseline

```c
lhdc_diag_set_modes(0, 0, 0, 0);
```

### Test IMDCT phase/unfold

```c
lhdc_diag_set_one("ref", 1);   // force direct reference IMDCT for N=960
lhdc_diag_set_one("ref", 0);   // back to fast IMDCT
```

If this cleans the artifact, the 960 fast IMDCT phase/unfold is still wrong.
This mode is slow; use it briefly for diagnosis, not release playback.

### Test SNS

```c
lhdc_diag_set_one("sns", 2);   // bypass SNS
lhdc_diag_set_one("sns", 1);   // multiply SNS instead of divide
lhdc_diag_set_one("sns", 0);   // current divide path
```

If bypassing SNS removes the fart/muffle, the remaining bug is SNS direction,
scale-factor reconstruction, or smoothing.

### Test coefficient reversal

```c
lhdc_diag_set_one("rev", 1);   // disable coeff reversal
lhdc_diag_set_one("rev", 0);   // enable coeff reversal again
```

If this changes low-frequency tone placement a lot, the low-bin ordering or nzc
interpretation is still wrong.

### Test OLA timing variant

```c
lhdc_diag_set_one("ola", 2);   // delayed shifted OLA variant
lhdc_diag_set_one("ola", 1);   // legacy full 50 percent OLA A/B
lhdc_diag_set_one("ola", 0);   // shifted low-overlap OLA baseline
```

If `ola=2` changes the artifact strongly, the remaining issue is still in the
short-overlap timing/latency rather than entropy or SNS.

## Suggested test sequence

For the 50 Hz artifact, do this exact sequence and note `same`, `cleaner`,
`worse`, or `gone`:

1. Baseline: `lhdc_diag_set_modes(0,0,0,0)`
2. `lhdc_diag_set_one("ref",1)` then back to `0`
3. `lhdc_diag_set_one("sns",2)` then back to `0`
4. `lhdc_diag_set_one("sns",1)` then back to `0`
5. `lhdc_diag_set_one("rev",1)` then back to `0`
6. `lhdc_diag_set_one("ola",2)` then back to `0`

Only change one variable at a time.
