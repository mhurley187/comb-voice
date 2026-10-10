import subprocess, numpy as np, os
SR = 48000
NAMES = ["Classic","Hollow","String","Metal","Thick","Cross"]
MUTS = ["None","Saturate","Fold","Crush","Clip","Decimate","Rectify"]

def render(t, m, v, fb, damp, freeze=0, pluck=1, secs=1.5, spread=0, chaos=0):
    subprocess.check_call(["./render", str(t), str(m), str(v), str(fb), str(damp), str(freeze), str(pluck), str(secs), "out.bin", str(spread), str(chaos)])
    d = np.fromfile("out.bin", dtype=np.float32)
    n = len(d)//2
    return d[:n], d[n:]

def est_pitch(x, lo=30, hi=3000):
    x = x - x.mean()
    w = x * np.hanning(len(x))
    N = 1<<18
    S = np.abs(np.fft.rfft(w, N))
    fr = np.fft.rfftfreq(N, 1/SR)
    ac = np.fft.irfft(S**2)[:len(x)//2]   # autocorrelation
    lags = np.arange(len(ac))
    lmin, lmax = int(SR/hi), int(SR/lo)
    k = lmin + np.argmax(ac[lmin:lmax])
    # parabolic refine
    y0,y1,y2 = ac[k-1],ac[k],ac[k+1]
    k = k + 0.5*(y0-y2)/(y0-2*y1+y2)
    return SR/k

print("== pitch accuracy (pluck, fb=0.97, damp=0.5, no mutation) -> cents error vs expected ==")
for t in range(6):
    row = []
    for v in (-1, 0, 1, 2):
        L,R = render(t, 0, v, 0.97, 0.5, 0, 1, 1.2)
        seg = L[int(0.1*SR):int(0.9*SR)]
        exp = 261.6256*2**v
        # hollow comb pitch estimate by autocorr is the true period when D=sr/(2f)*... fundamental f
        try:
            f = est_pitch(seg, lo=exp*0.6, hi=exp*1.7)
            row.append("%+6.1f" % (1200*np.log2(f/exp)))
        except Exception as e:
            row.append("  err ")
    print("%-8s  V=-1,0,1,2: %s" % (NAMES[t], "  ".join(row)))

print("\n== stability: fb=1.0 + freeze, every comb x mutation (peak, rms, finite?) ==")
bad = 0
for t in range(6):
    line = []
    for m in range(7):
        L,R = render(t, m, 0, 1.0, 0.4, 1, 1, 2.0)
        ok = np.all(np.isfinite(L)) and np.all(np.isfinite(R))
        pk = max(np.abs(L).max(), np.abs(R).max())
        rms = np.sqrt(np.mean(L[SR:]**2))
        if not ok or pk > 1.0001: bad += 1
        line.append("%.2f/%.2f" % (pk, rms))
    print("%-8s %s" % (NAMES[t], "  ".join(line)))
print("problems:", bad)
