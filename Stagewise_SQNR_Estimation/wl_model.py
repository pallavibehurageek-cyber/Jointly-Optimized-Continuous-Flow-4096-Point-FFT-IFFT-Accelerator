import numpy as np
# Radix-16^3 DIF model matching the RTL: each BS16 = 2 radix-4 levels, each
# scaled >>>1. Quantization applied at each stage's memory write (word length Wk).
N=4096; rng=np.random.default_rng(1)

def quant(x, W):
    """Truncate to W-bit signed fixed point (same LSB grid as 16-bit)."""
    if W>=16: return x
    step = 2**(16-W)
    return np.floor(x.real/step)*step + 1j*np.floor(x.imag/step)*step

def fft_staged(x, W1, W2, W3):
    """4096 = 16x16x16 DIF, /4 scaling per BS16 stage, quantized at stage outputs."""
    X = x.reshape(16,16,16)          # indices (n1,n2,n3)
    # Stage I: DFT along n1, twiddle, scale /4, quantize to W1
    X = np.fft.fft(X, axis=0)/4.0
    k1 = np.arange(16).reshape(16,1,1); n2 = np.arange(16).reshape(1,16,1); n3 = np.arange(16).reshape(1,1,16)
    X = X*np.exp(-2j*np.pi*k1*(16*n2+n3)/4096.0)
    X = quant(X, W1)
    # Stage II
    X = np.fft.fft(X, axis=1)/4.0
    k2 = np.arange(16).reshape(1,16,1)
    X = X*np.exp(-2j*np.pi*k2*n3/256.0)
    X = quant(X, W2)
    # Stage III
    X = np.fft.fft(X, axis=2)/4.0
    X = quant(X, W3)
    return X.transpose(2,1,0).reshape(-1)   # digit-reversed -> natural

def sqnr(W1,W2,W3,ntr=6,amp=2047):
    s=e=0.0
    for _ in range(ntr):
        x = (rng.integers(-amp,amp+1,N) + 1j*rng.integers(-amp,amp+1,N)).astype(complex)
        ref = fft_staged(x,16,16,16)          # same structure, no extra quantization
        exact = np.fft.fft(x)/64.0
        y = fft_staged(x,W1,W2,W3)
        s += np.sum(np.abs(exact)**2); e += np.sum(np.abs(y-exact)**2)
    return 10*np.log10(s/e)

print("Baseline uniform 16/16/16 : %.2f dB\n"%sqnr(16,16,16))

print("=== Which stage is most noise-sensitive? (drop ONE stage to 12 bits) ===")
for tag,(a,b,c) in [("W1=12 (early)",(12,16,16)),("W2=12 (mid)",(16,12,16)),("W3=12 (late)",(16,16,12))]:
    print("  %-16s %.2f dB"%(tag,sqnr(a,b,c)))

print("\n=== Candidate profiles ===")
for p in [(16,16,16),(14,14,14),(12,12,12),(12,14,16),(14,15,16),(13,14,16),(12,13,14)]:
    print("  %-12s %.2f dB"%(str(p),sqnr(*p)))
