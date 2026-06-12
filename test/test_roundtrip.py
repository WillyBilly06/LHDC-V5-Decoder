import sys
sys.path.insert(0, '.')
from rice_dec import rice_decode

qcoeff = [int(x) for x in open('../testvectors/qcoeff_br7.txt').read().split()]
count = len(qcoeff)
print(f"QCOEFF count={count}")

for split in [0, 1920]:
    tern = [int(x) for x in open(f'../testvectors/tern_split{split}.txt').read().split()]
    s1 = tern[:count]
    s2 = tern[count:]
    coeff, used = rice_decode(s1, s2, count, split)
    ok = (coeff == qcoeff)
    print(f"\nsplit={split}: ternlen={len(tern)} s2used={used} roundtrip={'EXACT' if ok else 'MISMATCH'}")
    if not ok:
        # show first mismatch
        for i in range(count):
            if coeff[i] != qcoeff[i]:
                print(f"  first mismatch at {i}: decoded={coeff[i]} truth={qcoeff[i]}")
                print(f"  decoded[{i}:{i+12}]={coeff[i:i+12]}")
                print(f"  truth  [{i}:{i+12}]={qcoeff[i:i+12]}")
                break
