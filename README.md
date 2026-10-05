# Jointly-Optimized-Continuous-Flow-4096-Point-FFT-IFFT-Accelerator for 5G

This repository contains the RTL, a bit-accurate reference model, verification
infrastructure, analysis scripts, and synthesis reports for three coordinated
micro-architectural optimizations applied to a ROM-free, conflict-free
radix-16<sup>3</sup> continuous-flow 4096-point FFT/IFFT processor for 5G
baseband.

---

## Overview

Starting from a faithfully reproduced ROM-free, conflict-free baseline, three
optimizations jointly reduce idle-cycle energy, active-cycle latency, and
storage precision **without changing the underlying dataflow**:

- **Occupancy-driven clock gating** — the controller's existing occupancy
  delay line is reused as an *exact* per-stage liveness signal driving eight
  integrated clock-gating domains. Proven bit-identical to the ungated design.
- **Reduced-cycle CORDIC** — six coarse micro-rotations plus one small-angle
  linear correction stage replace ten iterations, exploiting the finite,
  statically known inter-stage twiddle angle sets. ROM-free is preserved.
- **Uniform memory word-length reduction** — an exhaustive stage-wise search
  shows a uniform profile is Pareto-optimal for this scaling policy.

## Key results

Measured against the reproduced baseline under an identical 45 nm synthesis
flow. Headline configuration: 13-bit memory.

| Metric | Baseline | Optimized | Change |
|---|---|---|---|
| Latency (cycles) | 605 | 558 | -7.8% |
| System SQNR (dB) | 50.49 | 54.88 | +4.4 dB |
| Total area (um^2) | 3,961,857 | 3,206,082 | -19.08% |
| Power @ 100% occupancy (mW) | 267.7 | 121.3 | -54.7% |
| Power @ 6.25% occupancy (mW) | 206.0 | 17.9 | -91.3% |
| Clock-gating coverage | -- | 99.8% | 0.002% area cost |
| Functional tests | 15/15 | 15/15 | bit-identical gating |

---

## RTL variants

| Variant | Gating | CORDIC | Padding | Memory | Latency |
|---|---|---|---|---|---|
| `paper_baseline` | none | 10-iter | present | 16-bit | 605 |
| `without_wordlength_opt` | 8 ICG | 6+fine | removed | 16-bit | 558 |
| `final_novelty` | 8 ICG | 6+fine | removed | `W_MEM` bits | 558 |

The `final_novelty` top module exposes `W_MEM1/2/3`; the reported sweep uses a
uniform 12/13/14/16-bit setting.

## Reproducing the results

### Functional verification and SQNR (open-source tools)
Any IEEE-1364/1800 simulator works. Example with Icarus Verilog on the
`final_novelty` variant:
```bash
cd rtl/final_novelty
iverilog -g2012 -o sim_num *.v            # includes tb_fft4096_numerical
vvp sim_num                                # expects 15/15 PASS
```
System-level SQNR against the double-precision reference:
```bash
iverilog -g2012 -o sim_sqnr <rtl>.v tb_sqnr.v
vvp sim_sqnr
python3 ../../analysis/sqnr.py .
```
Memory word length is set by the `W_MEM1/2/3` parameters in the top module.

### Word-length exploration (no EDA tools)
```bash
python3 analysis/wordlength_explorer.py 13 13 13    # per-stage SQNR + bits
python3 analysis/wordlength_explorer.py --pareto     # exhaustive Pareto front
```

### Reference model (MATLAB / Octave)
```matlab
cd reference_model
generate_all_test_vectors    % builds vectors/ and expected/
verify_architecture          % end-to-end bit-accurate check
```

### Synthesis and power (commercial tool)
`synthesis/scripts/*.tcl` target Cadence Genus with a 45 nm standard-cell
library. Power is annotated from SAIF captured in simulation; each script
documents the simulate-then-annotate flow. Pre-generated reports are in
`synthesis/reports/`.

## Report naming

Within each `synthesis/reports/novelty_wNN/` folder:
`area_hier`, `cg_mapping`, `clock_gating`, `gates`, `power_vectorless_hier`,
`qor`, `timing`. The `power_saif/` folders hold the occupancy sweep, where
`occ100` = 100% frame occupancy down to `occ0063` = 6.25%.

## Reproducibility notes and limitations

- **Technology.** Synthesis uses a 45 nm standard-cell library. All comparisons
  are against the identically configured reproduced baseline, not against
  absolute figures from any prior publication.
- **Memory.** Storage is synthesized from standard cells, not compiled SRAM
  macros; word-length area savings reflect register-array scaling and would be
  smaller with hardened macros.
- **Power.** Reported at the pre-clock-tree stage, so the gating benefit is a
  lower bound.
- **SQNR.** The reduced-CORDIC SQNR improvement includes the effect of
  corrected rotation constants.
- **Cross-simulator.** Functional results were confirmed identical across two
  independent simulators.

## License

Released under the MIT License. See [`LICENSE`](LICENSE).
