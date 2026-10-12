import numpy as np, sys

def rd(p):
    v=[]
    for ln in open(p):
        ln=ln.strip()
        if not ln or ln.startswith('/'): continue
        for tok in ln.split():
            x=int(tok,16)
            if x>=0x8000: x-=0x10000
            v.append(x)
    return np.array(v,dtype=float)

d=sys.argv[1] if len(sys.argv)>1 else '.'
ire=rd(f'{d}/sq_ire.hex'); iim=rd(f'{d}/sq_iim.hex')
ore=rd(f'{d}/sq_ore.hex'); oim=rd(f'{d}/sq_oim.hex')
NF=len(ire)//4096
print(f"frames={NF}")
GAIN=1/64.0     # architecture scales by 2^-6
tot_s=tot_e=0.0
for f in range(NF):
    x=ire[f*4096:(f+1)*4096]+1j*iim[f*4096:(f+1)*4096]
    yr=ore[f*4096:(f+1)*4096]; yi=oim[f*4096:(f+1)*4096]
    # storage order is (cycle,lane); FFT bin = lane*256 + cycle
    y=np.zeros(4096,dtype=complex)
    idx=np.arange(4096); cyc=idx//16; lane=idx%16
    y[lane*256+cyc]=yr+1j*yi
    ref=np.fft.fft(x)*GAIN
    e=y-ref
    s=np.sum(np.abs(ref)**2); er=np.sum(np.abs(e)**2)
    tot_s+=s; tot_e+=er
    print(f"  frame {f}: SQNR = {10*np.log10(s/er):6.2f} dB   "
          f"ref_peak={np.max(np.abs(ref)):7.1f}  rtl_peak={np.max(np.abs(y)):7.1f}")
print(f"OVERALL SYSTEM SQNR = {10*np.log10(tot_s/tot_e):.2f} dB")
