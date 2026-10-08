#!/usr/bin/env python3
# =============================================================================
# IEEE-style figures for the FFT co-optimization paper.
# Produces vector PDFs sized for a single IEEE column (~3.5 in wide).
# =============================================================================
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FixedFormatter

# ---- IEEE-matching style -----------------------------------------------------
# IEEE uses Times. Prefer Times New Roman; fall back to a serif Times clone so
# the script runs anywhere. mathtext 'stix' matches Times-style math glyphs.
plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman", "Nimbus Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size":        8,      # IEEE body text is ~8 pt in figures
    "axes.labelsize":   8,
    "axes.titlesize":   8,
    "legend.fontsize":  7,
    "xtick.labelsize":  7,
    "ytick.labelsize":  7,
    "axes.linewidth":   0.6,
    "lines.linewidth":  1.1,
    "lines.markersize": 4,
    "legend.frameon":   True,
    "legend.framealpha":1.0,
    "legend.edgecolor": "black",
    "legend.fancybox":  False,
    "pdf.fonttype":     42,     # embed TrueType so reviewers' viewers render it
    "ps.fonttype":      42,
})

COL_W = 3.5          # inches, one IEEE column
IEEE_BLUE = "#0072BD"
IEEE_RED  = "#D95319"

# =============================================================================
# Figure 1 : Power (normalized to full-rate) vs frame occupancy
# =============================================================================
occ      = [100, 50, 25, 12.5, 6.25]
baseline = [100, 92.5, 86.0, 80.8, 77.0]
gated    = [100, 88.6, 50.2, 27.1, 14.7]

fig, ax = plt.subplots(figsize=(COL_W, COL_W * 0.72))
ax.set_xscale("log", base=2)

ax.plot(occ, baseline, marker="s", color="black",
        label="Baseline (ungated)", markerfacecolor="white",
        markeredgewidth=0.9, clip_on=False, zorder=3)
ax.plot(occ, gated, marker="o", color=IEEE_RED,
        label="Optimized (gated)", markerfacecolor=IEEE_RED,
        markeredgewidth=0.9, clip_on=False, zorder=3)

ax.set_xlabel("Frame occupancy (\\%)" if plt.rcParams["text.usetex"]
              else "Frame occupancy (%)")
ax.set_ylabel("Power, normalized to 100\\% rate" if plt.rcParams["text.usetex"]
              else "Power, normalized to 100% rate (%)")
ax.set_xlim(5.5, 105)
ax.set_ylim(0, 105)
ax.xaxis.set_major_locator(FixedLocator(occ))
ax.xaxis.set_major_formatter(FixedFormatter([f"{o:g}" for o in occ]))
ax.set_yticks(range(0, 101, 20))
ax.grid(True, which="major", linestyle=":", linewidth=0.4, color="gray", alpha=0.7)
ax.tick_params(direction="in", length=3, width=0.6, top=True, right=True)
ax.legend(loc="lower right", handlelength=1.8, borderpad=0.4, labelspacing=0.3)
for s in ax.spines.values():
    s.set_linewidth(0.6)
fig.tight_layout(pad=0.3)
fig.savefig("fig_power_occupancy.pdf", bbox_inches="tight", pad_inches=0.02)
fig.savefig("fig_power_occupancy.png", dpi=300, bbox_inches="tight", pad_inches=0.02)
print("wrote fig_power_occupancy.pdf / .png")

# =============================================================================
# Figure 2 : System SQNR vs uniform memory word length
# =============================================================================
W    = [16, 15, 14, 13, 12]
sqnr = [54.88, 52.74, 48.10, 42.25, 36.32]

fig2, ax2 = plt.subplots(figsize=(COL_W, COL_W * 0.72))
ax2.plot(W, sqnr, marker="o", color=IEEE_BLUE, markerfacecolor=IEEE_BLUE,
         markeredgewidth=0.9, clip_on=False, zorder=3, label="RTL measured")
ax2.axhline(30, linestyle="--", color=IEEE_RED, linewidth=1.0,
            label="256-QAM requirement")

# annotate the recommended operating point
ax2.annotate("13-bit:\n42.25 dB", xy=(13, 42.25), xytext=(13.4, 34.5),
             fontsize=6.5, ha="left",
             arrowprops=dict(arrowstyle="->", lw=0.6))

ax2.set_xlabel("Memory word length (bits)")
ax2.set_ylabel("System SQNR (dB)")
ax2.set_xlim(11.6, 16.4)
ax2.set_ylim(28, 58)
ax2.set_xticks(W)
ax2.set_yticks(range(30, 56, 5))
ax2.grid(True, which="major", linestyle=":", linewidth=0.4, color="gray", alpha=0.7)
ax2.tick_params(direction="in", length=3, width=0.6, top=True, right=True)
ax2.legend(loc="upper left", handlelength=1.8, borderpad=0.4, labelspacing=0.3)
for s in ax2.spines.values():
    s.set_linewidth(0.6)
fig2.tight_layout(pad=0.3)
fig2.savefig("fig_sqnr_wordlength.pdf", bbox_inches="tight", pad_inches=0.02)
fig2.savefig("fig_sqnr_wordlength.png", dpi=300, bbox_inches="tight", pad_inches=0.02)
print("wrote fig_sqnr_wordlength.pdf / .png")
