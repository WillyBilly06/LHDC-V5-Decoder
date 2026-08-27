# LHDC V5 96k/192k decoder fixes

This package contains a best-effort patch set for the current decoder files you uploaded.

## Main fixes

1. **96k/192k synthesis window bug**
   - The decoder returned const HR windows before the runtime generator.
   - The const 960 table is zero=152 / repeating=176.
   - The corrected generated 960 window is zero=144 / repeating=192.
   - The const 1920 table is zero=304 / repeating=352.
   - The corrected generated 1920 window is zero=288 / repeating=384.
   - This mismatch is a likely source of muffled/dirty low-frequency output and TDAC leakage.
   - Patch: `lhdc_get_window_const()` now returns NULL for 960/1920 so `lhdc_gen_kbd_window()` builds the correct HR window in RAM.

2. **Flush only cleared a pointer-sized amount of overlap**
   - `lhdc_dec_flush()` used `sizeof(dec->overlap_buf[ch])`, which clears only 4/8 bytes.
   - This leaves stale overlap after flush/reset/underrun and can sound like low-frequency leakage.
   - Patch: clears `(alloc_mdct_size / 2) * sizeof(float)` per channel.

3. **Entropy stream-2 decoder CPU reduction**
   - Old path predecoded up to `2*count` stream-2 symbols and then decoded stream-2 again to recover `fac_bytes`.
   - Patched path uses the existing on-demand stream reader and returns the FAC byte/range state directly.
   - This should reduce 96k/900kbps underruns and lower stutter pressure.

4. **192k sizing support**
   - `LHDC_DEC_MAX_MDCT_SIZE` changed 960 -> 1920.
   - `LHDC_DEC_MAX_FRAME_BYTES` changed 768 -> 1536.
   - `lhdc_get_mdct_size()` now returns the transform size from `lhdc_get_band_cfg()`, not samples-per-frame.

5. **Safer selector retry signal**
   - `g_lhdc_chan_specmax` is now actually populated.
   - The auto/flip descramble selector retry also checks `g_lhdc_chan_maxM`, not only final PCM peak.

## Quick sanity checks performed

Built all C files with:

```bash
gcc -DLHDC_HOST_BUILD -Wall -Wextra -O2 -I. -c lhdc_bit_reader.c lhdc_entropy_dec.c lhdc_imdct.c lhdc_sns_synth.c lhdc_tables.c lhdc_dec.c -lm
```

Window check after patch:

```text
N=480  const=yes  first=76   last=403   ones=152  maxPB=1.19e-7
N=960  const=no   first=144  last=815   ones=292  maxPB=2.38e-7
N=1920 const=no   first=288  last=1631  ones=582  maxPB=3.58e-7
```

IMDCT-960 fast path was also checked against the direct reference formula on a random spectrum:

```text
max relative error ~= 5.8e-7
```

## Files changed

- `lhdc_dec_internal.h`
- `lhdc_tables.c`
- `lhdc_dec.c`
- `lhdc_entropy_dec.c`
