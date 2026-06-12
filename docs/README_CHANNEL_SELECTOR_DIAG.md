# LHDC low-frequency remaining-artifact diagnostics

This build keeps the improved shifted OLA path and adds runtime tests for the remaining 50 Hz "fart/leak" artifact.

Use your existing serial command wrapper if it forwards unknown keys to `lhdc_diag_set_one(key,value)`. If your wrapper hard-validates keys, add `out`, `sel0`, and `sel1`.

Commands:

```text
lhdc out 0   # normal stereo
lhdc out 1   # decoded channel 0 copied to L/R
lhdc out 2   # decoded channel 1 copied to L/R
lhdc out 3   # left only
lhdc out 4   # right only
lhdc out 5   # swap L/R

lhdc sel0 0  # ch0 auto descramble selector
lhdc sel0 1  # ch0 force selector 0
lhdc sel0 2  # ch0 force selector 1
lhdc sel1 0  # ch1 auto descramble selector
lhdc sel1 1  # ch1 force selector 0
lhdc sel1 2  # ch1 force selector 1
```

Test order with the same 50 Hz / 96 kHz stream:

```text
lhdc out 1
lhdc out 2
lhdc out 0

lhdc sel0 1
lhdc sel0 2
lhdc sel0 0

lhdc sel1 1
lhdc sel1 2
lhdc sel1 0
```

Interpretation:

- If `out 1` is clean but `out 2` has the artifact: ch1 slice/selector/entropy alignment is the issue.
- If `out 2` is clean but `out 1` has it: ch0 alignment is the issue.
- If both `out 1` and `out 2` have it: the issue is common transform/dequant/SNS.
- If force selector 0 or 1 cleans up one channel, disable the peak-based autosel for that channel and use that selector rule.
