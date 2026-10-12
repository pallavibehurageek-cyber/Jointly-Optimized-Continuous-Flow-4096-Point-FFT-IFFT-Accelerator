import numpy as np, itertools
exec(open('wl_model.py').read().split('print("Baseline')[0])

# Area model: memory dominates. Each stage = 16 banks x 256 deep x 2 sets,
# complex (re+im) -> bits proportional to Wk. Equal weight per stage here.
def area(W): return sum(W)          # relative memory-bit cost

print("=== EXHAUSTIVE Pareto search over (W1,W2,W3), 10..16 bits = 343 configs ===")
cfgs=[]
for W in itertools.product(range(10,17),repeat=3):
    cfgs.append((W, sqnr(*W, ntr=3), area(W)))

# Pareto front: maximize SQNR, minimize area
front=[]
for W,s,a in cfgs:
    if not any((s2>=s and a2<=a and (s2>s or a2<a)) for W2,s2,a2 in cfgs):
        front.append((W,s,a))
front.sort(key=lambda t:t[2])
print(f"{'profile':>14} {'bits':>5} {'SQNR':>8}   uniform?")
for W,s,a in front:
    print(f"{str(W):>14} {a:>5} {s:>7.2f} dB   {'YES' if len(set(W))==1 else 'no'}")
