import numpy as np, math
# MDCT/IMDCT round-trip with the decoder's actual synthesis window (gen, repeating=192).
# Tests: input PCM tone -> analysis MDCT -> decoder IMDCT -> output PCM == input (1:1)?
def gen_win(N, repeating, num4):
    w=np.zeros(N)
    for i in range(N):
        if i<num4 or i>=N-num4: w[i]=0.0
        elif i<num4+repeating:
            j=i-num4; s=math.sin((j+0.5)*(math.pi/2)/repeating); w[i]=math.sin((math.pi/2)*s*s)
        elif i>=N-num4-repeating:
            j=(N-1-i)-num4; s=math.sin((j+0.5)*(math.pi/2)/repeating); w[i]=math.sin((math.pi/2)*s*s)
        else: w[i]=1.0
    return w
def run(N, fs, repeating, num4, label):
    M=N//2; H=N//2
    w=gen_win(N,repeating,num4)
    n=np.arange(N); k=np.arange(M)
    C=np.cos((math.pi/N)*np.outer((n+0.5+N/4.0),(2*k+1)))   # (N,M) basis
    print(f"\n== {label}: N={N} fs={fs} repeating={repeating} window PR(w^2+w^2)max-dev={np.max(np.abs(w[:H]**2+w[H:]**2-1)):.2e} ==")
    print("   f_in(Hz)  f_out(Hz)  err   in-band%  leakage(dB)")
    for fin in [100,500,1000,2000,4000,8000,12000,16000,20000,22000]:
        if fin>=fs/2: continue
        L=200*H
        t=np.arange(L)/fs; x=np.sin(2*math.pi*fin*t)
        y=np.zeros(L+N); ov=np.zeros(H)
        nf=(L-N)//H
        for m in range(nf):
            seg=x[m*H:m*H+N]*w
            X=seg@C                       # analysis MDCT (N->M)
            rec=(C@X)*(2.0/N)             # decoder IMDCT (M->N), matches lhdc_imdct_transform
            rec*=w                        # synthesis window
            y[m*H:m*H+H]=rec[:H]+ov; ov=rec[H:]
        out=y[N:nf*H]                     # steady region, drop startup latency
        inp=x[N:nf*H]
        # spectra
        W=np.hanning(len(out)); F=np.fft.rfftfreq(len(out),1/fs)
        Xo=np.abs(np.fft.rfft(out*W)); Xi=np.abs(np.fft.rfft(inp*W))
        fo=F[np.argmax(Xo)]
        bm=np.argmin(np.abs(F-fin)); inb=np.sum(Xo[max(0,bm-3):bm+4]**2); tot=np.sum(Xo**2)
        leak_db=10*np.log10(max(1e-20,(tot-inb)/tot))
        print(f"   {fin:7d}  {fo:8.1f}  {fo-fin:+5.1f}   {100*inb/tot:6.2f}   {leak_db:7.1f}")
run(480,48000,88,76,"48k (solved window, verified clean)")
run(960,96000,192,144,"96k (decoder's current gen window, repeating=192)")
run(960,96000,176,152,"96k (old table window, repeating=176)")
