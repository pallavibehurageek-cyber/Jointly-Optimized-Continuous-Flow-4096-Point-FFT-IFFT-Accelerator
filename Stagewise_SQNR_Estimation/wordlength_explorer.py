#!/usr/bin/env python3
# =============================================================================
#  Per-stage word-length explorer for the radix-16^3 continuous-flow FFT
# =============================================================================
#  Calibrated fixed-point model. Replicates EVERY quantizer in the RTL:
#    - BS16 = two radix-4 levels; each does floor(sum4 / 2)        (>>>1)
#    - intra-stage CSD twiddle multiply, floor at 2^-15            (>>>15)
#    - CORDIC inter-stage rotation, floored to the 16-bit grid
#    - memory write truncated to W1 / W2 / W3 bits                 (novelty #4)
#
#  CALIBRATION: model = 56.49 dB vs RTL-measured 54.88 dB at (16,16,16).
#  The model runs ~1.6 dB OPTIMISTIC. Subtract ~1.6 dB to predict RTL.
#
#  USAGE
#    python3 wordlength_explorer.py                 # uses CONFIG below
#    python3 wordlength_explorer.py 13 13 13        # explicit W1 W2 W3
#    python3 wordlength_explorer.py --sweep         # uniform sweep 16..10
#    python3 wordlength_explorer.py --sensitivity   # per-stage sensitivity
#    python3 wordlength_explorer.py --pareto        # exhaustive Pareto front
# =============================================================================

import sys, itertools
import numpy as np

# ----------------------------- CONFIG ---------------------------------------
W1, W2, W3 = 13, 13, 13     # <-- CHANGE THESE: memory word length per stage
NTRIALS    = 4              # random frames averaged (more = smoother)
AMP        = 2047           # input amplitude (2047 is the design's sweet spot)
# -----------------------------------------------------------------------------

# Memory geometry (from the RTL): per stage, 16 banks x 256 deep x 2 sets,
# complex (re + im). Bits per stage = 16*256*2*2*W = 16384*W
BANKS, DEPTH, SETS, CPLX = 16, 256, 2, 2
BITS_PER_STAGE_PER_W = BANKS * DEPTH * SETS * CPLX      # 16384

rng = np.random.default_rng(7)


def fl(z):
    """Floor truncation on both components - matches Verilog >>> on signed."""
    return np.floor(z.real) + 1j * np.floor(z.imag)


def trunc_bits(z, W):
    """Keep W of 16 bits (drop the 16-W LSBs), floor. W>=16 is a no-op."""
    if W >= 16:
        return z
    s = 2 ** (16 - W)
    return (np.floor(z.real / s) * s) + 1j * (np.floor(z.imag / s) * s)


def dft4_scaled(X, axis):
    """One radix-4 butterfly followed by the RTL's >>>1 scaling."""
    return fl(np.fft.fft(X, axis=axis) / 2.0)


def bs16(X, axis):
    """16-point DFT as two radix-4 levels, with the RTL's internal truncations."""
    sh = list(X.shape)
    X = X.reshape(sh[:axis] + [4, 4] + sh[axis + 1:])
    a = axis
    X = dft4_scaled(X, a)                                  # radix-4 level 1
    n1 = np.arange(4).reshape([4, 1])
    k2 = np.arange(4).reshape([1, 4])
    tw = np.exp(-2j * np.pi * n1 * k2 / 16.0)
    shp = [1] * len(X.shape); shp[a] = 4; shp[a + 1] = 4
    X = fl(X * tw.reshape(shp) * (2 ** 15)) / (2 ** 15)    # CSD twiddle, >>>15
    X = dft4_scaled(X, a + 1)                              # radix-4 level 2
    X = np.swapaxes(X, a, a + 1)                           # digit reversal
    return X.reshape(sh)


def fft4096(x, w1, w2, w3):
    X = x.reshape(16, 16, 16)
    k1 = np.arange(16).reshape(16, 1, 1)
    n2 = np.arange(16).reshape(1, 16, 1)
    n3 = np.arange(16).reshape(1, 1, 16)
    # ---- Stage I ----
    X = bs16(X, 0)
    X = fl(X * np.exp(-2j * np.pi * k1 * (16 * n2 + n3) / 4096.0))   # CORDIC
    X = trunc_bits(X, w1)                                            # memory
    # ---- Stage II ----
    X = bs16(X, 1)
    k2 = np.arange(16).reshape(1, 16, 1)
    X = fl(X * np.exp(-2j * np.pi * k2 * n3 / 256.0))                # CORDIC
    X = trunc_bits(X, w2)
    # ---- Stage III ---- (no CORDIC)
    X = bs16(X, 2)
    X = trunc_bits(X, w3)
    return X.transpose(2, 1, 0).reshape(-1)


def sqnr(w1=16, w2=16, w3=16, ntr=NTRIALS, amp=AMP):
    """System SQNR (dB) vs a double-precision FFT reference."""
    s = e = 0.0
    for _ in range(ntr):
        x = (rng.integers(-amp, amp + 1, 4096)
             + 1j * rng.integers(-amp, amp + 1, 4096)).astype(complex)
        ex = np.fft.fft(x) / 64.0          # architecture scales by 2^-6
        y = fft4096(x, w1, w2, w3)
        s += np.sum(np.abs(ex) ** 2)
        e += np.sum(np.abs(y - ex) ** 2)
    return 10 * np.log10(s / e)


def mem_bits(w1, w2, w3):
    """Total FFT memory bits for a given per-stage word-length profile."""
    return BITS_PER_STAGE_PER_W * (w1 + w2 + w3)


def report(w1, w2, w3):
    base = mem_bits(16, 16, 16)
    bits = mem_bits(w1, w2, w3)
    s = sqnr(w1, w2, w3)
    print(f"  profile (W1,W2,W3)   : ({w1}, {w2}, {w3})")
    print(f"  system SQNR (model)  : {s:6.2f} dB")
    print(f"  predicted RTL SQNR   : {s-1.6:6.2f} dB   (model runs ~1.6 dB optimistic)")
    print(f"  memory bits          : {bits:,} of {base:,}")
    print(f"  memory bits SAVED    : {base-bits:,}  ({100.0*(base-bits)/base:.2f} %)")
    print(f"  NOTE: bit count is exact; SILICON AREA saving is smaller and")
    print(f"        unknown until synthesis (SRAM periphery does not scale).")
    return s, bits


if __name__ == "__main__":
    args = [a for a in sys.argv[1:]]

    if "--sweep" in args:
        print("=== UNIFORM word-length sweep ===")
        base = mem_bits(16, 16, 16)
        print(f"{'W':>4} {'SQNR(model)':>12} {'SQNR(pred RTL)':>15} {'bits saved':>12}")
        for W in range(16, 9, -1):
            s = sqnr(W, W, W)
            sv = 100.0 * (base - mem_bits(W, W, W)) / base
            print(f"{W:>4} {s:>9.2f} dB {s-1.6:>12.2f} dB {sv:>11.2f} %")

    elif "--sensitivity" in args:
        print("=== PER-STAGE SENSITIVITY (drop one stage to 12 bits) ===")
        print(f"  baseline (16,16,16) : {sqnr(16,16,16):.2f} dB")
        for tag, c in [("W1=12 (early)", (12, 16, 16)),
                       ("W2=12 (mid)",   (16, 12, 16)),
                       ("W3=12 (late)",  (16, 16, 12))]:
            print(f"  {tag:<16} {sqnr(*c):.2f} dB")

    elif "--pareto" in args:
        print("=== EXHAUSTIVE Pareto front over (W1,W2,W3), 10..16 ===")
        cfg = [(p, sqnr(*p, ntr=3), sum(p)) for p in itertools.product(range(10, 17), repeat=3)]
        front = [c for c in cfg
                 if not any((s2 >= c[1] and a2 <= c[2] and (s2 > c[1] or a2 < c[2]))
                            for _, s2, a2 in cfg)]
        front.sort(key=lambda t: t[2])
        print(f"{'profile':>14} {'bits':>5} {'SQNR':>10}  uniform?")
        for p, s, a in front:
            print(f"{str(p):>14} {a:>5} {s:>7.2f} dB  {'YES' if len(set(p))==1 else 'no'}")

    # else:
    #     nums = [int(a) for a in args if a.isdigit()]
    #     if len(nums) == 3:
    #         w1, w2, w3 = nums
    #     elif len(nums) == 1:
    #         w1 = w2 = w3 = nums[0]
    #     else:
    #         w1, w2, w3 = W1, W2, W3
    #     print("=== Per-stage word-length evaluation ===")
    #     report(w1, w2, w3)
    else:
        print("=== Exhaustive evaluation of all word-length combinations ===")

        base_bits = mem_bits(16, 16, 16)

        print(f"{'W1':>3} {'W2':>3} {'W3':>3} "
                f"{'SQNR(model)':>12} {'RTL(pred)':>10} "
                f"{'Memory(bits)':>14} {'Saved(%)':>10}")

        for w1 in range(12, 17):
            for w2 in range(12, 17):
                for w3 in range(12, 17):

                    s = sqnr(w1, w2, w3)
                    bits = mem_bits(w1, w2, w3)
                    saved_pct = 100.0 * (base_bits - bits) / base_bits

                    print(
                        f"{w1:>3} {w2:>3} {w3:>3} "
                        f"{s:>10.2f} dB "
                        f"{s-1.6:>8.2f} dB "
                        f"{bits:>14,} "
                        f"{saved_pct:>9.2f}%"
                    )
