# LHDC 96 kHz low-frequency artifact diagnostic build

This package is based on the shifted low-overlap OLA build, which sounded better than the original but still left a low-frequency 'fart/muffle' artifact on a 50 Hz tone.

The remaining artifact needs to be isolated before another blind fix. The file `lhdc_diag_config.h` adds compile-time switches. Change only one switch at a time, rebuild, and test the same 50 Hz / 96 kHz / 24-bit stream.

## Default

```
#define LHDC_DIAG_FORCE_REF_IMDCT_960 0
#define LHDC_DIAG_SNS_MODE 0
#define LHDC_DIAG_DISABLE_COEFF_REV 0
#define LHDC_DIAG_OLA_MODE 0
```

This is the current best shifted-OLA path.

## Test order

### Test 1: Force exact 960 IMDCT

```
#define LHDC_DIAG_FORCE_REF_IMDCT_960 1
```

If the artifact disappears, the fast 960 FFT unfold/phase is still wrong. This is too slow for release, but it proves the bug location.

### Test 2: Bypass SNS

Set IMDCT back to 0, then:

```
#define LHDC_DIAG_SNS_MODE 2
```

If the fart/noise disappears but tonal balance is wrong, the remaining bug is SNS side-info, post-smoothing, or inverse direction.

Optional SNS direction test:

```
#define LHDC_DIAG_SNS_MODE 1
```

This multiplies by the SNS gain instead of dividing.

### Test 3: Coefficient reversal

Set SNS back to 0, then:

```
#define LHDC_DIAG_DISABLE_COEFF_REV 1
```

If 50 Hz changes a lot, low-bin coefficient ordering or the interpretation of `nzc` is wrong.

### Test 4: OLA latency hypothesis

Set coeff reversal back to 0, then:

```
#define LHDC_DIAG_OLA_MODE 2
```

This tests a more original-like delayed low-overlap transition. It may sound delayed/odd, but if the fart artifact changes strongly, the remaining mismatch is still OLA timing.

## What to report

For each test, report only one of:

- same artifact
- cleaner but tonal balance changed
- worse
- artifact gone

The most useful result is which single switch changes the 50 Hz fart/leakage the most.
