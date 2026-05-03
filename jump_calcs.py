"""
jump_calcs.py — refined jump kinematics using frame-by-frame video measurements.

Recording: 240 fps camera, played back at 30 fps.
Each playback frame = 1/240 s real time.
Frame 0 = spring release ('d' pressed).
Liftoff = frame 9 (end of 9-frame ground contact).

Note: video and IMU data are from separate runs (different springs).
"""

import numpy as np
import matplotlib.pyplot as plt

# ── Constants ─────────────────────────────────────────────────────────────────
fps_record = 240
mass_kg    = 0.3
g          = 9.81

# ── Measured data (frame number, height cm) ───────────────────────────────────
# Ascent
meas_frames = np.array([ 0, 15, 18, 23, 28,  35,  45,  60,   # ascent
                         76, 85, 93,101, 109, 117])            # descent
meas_h_cm   = np.array([ 0,  4,  8, 12, 16,  20,  24,  26,
                         24, 22, 18, 14,  10,   0])
meas_t      = meas_frames / fps_record   # real time from spring release (s)

# ── Launch phase ──────────────────────────────────────────────────────────────
frame_liftoff = 9
t_liftoff     = frame_liftoff / fps_record   # 0.0375 s
t_contact     = t_liftoff                    # spring contact duration

# ── Fit parabola to free-flight data ─────────────────────────────────────────
# Use all points from frame 15 onward (clearly post-liftoff).
# tau = time since liftoff; reference point is on the ground at liftoff → h(0)=0
# Constrained fit: h = A*tau² + B*tau  (no constant term)
ff_mask  = meas_frames >= 15
tau_ff   = meas_t[ff_mask] - t_liftoff       # time since liftoff (s)
h_ff     = meas_h_cm[ff_mask] / 100.0        # metres

# Least-squares with design matrix [tau², tau] — forces h(0)=0
X        = np.column_stack([tau_ff**2, tau_ff])
coeffs, _, _, _ = np.linalg.lstsq(X, h_ff, rcond=None)
A_fit, B_fit = coeffs

g_empirical = -2 * A_fit
v0          = B_fit       # m/s  launch velocity
h_liftoff   = 0.0         # m    reference point is on the ground at liftoff

# Polynomial array matching np.polyval convention [A, B, 0]
p = np.array([A_fit, B_fit, 0.0])

# ── Kinematics ────────────────────────────────────────────────────────────────
t_peak_fit       = v0 / g                        # time from liftoff to peak
h_peak_fit       = v0**2 / (2 * g)              # predicted peak height
t_air_measured   = (117 - frame_liftoff) / fps_record  # measured flight time
t_air_ballistic  = 2 * v0 / g                   # ballistic prediction

# ── Dynamics ──────────────────────────────────────────────────────────────────
avg_launch_accel = v0 / t_contact
weight_N         = mass_kg * g
avg_launch_force = mass_kg * (avg_launch_accel + g)
impulse_Ns       = mass_kg * v0
ke_liftoff       = 0.5 * mass_kg * v0**2
h_peak_measured  = 0.26   # m  (directly measured at frame 60)
pe_peak          = mass_kg * g * h_peak_measured

# ── Print table ───────────────────────────────────────────────────────────────
rows = [
    ("Input",      "Measured data points",           f"{len(meas_frames)} frames (ascent + descent)"),
    ("Input",      "Peak height (frame 60)",          f"{h_peak_measured*100:.0f} cm"),
    ("Input",      "Contact / liftoff (frame 9)",     f"{t_contact*1000:.1f} ms"),
    ("Input",      "Landing (frame 117)",             f"{117/fps_record*1000:.1f} ms from release"),
    ("Input",      "Robot mass",                      f"{mass_kg*1000:.0f} g"),
    ("",           "",                                ""),
    ("Fit",        "Empirical g from constrained fit", f"{g_empirical:.2f} m/s²  (expect 9.81)"),
    ("Fit",        "Launch velocity v0 (h=0 at liftoff)",f"{v0:.3f} m/s"),
    ("",           "",                                ""),
    ("Kinematics", "Launch velocity",                 f"{v0:.3f} m/s"),
    ("Kinematics", "Predicted peak height",           f"{h_peak_fit*100:.1f} cm  (measured: {h_peak_measured*100:.0f} cm)"),
    ("Kinematics", "Predicted time to peak",          f"{t_peak_fit*1000:.1f} ms  (measured: {(60-9)/fps_record*1000:.1f} ms)"),
    ("Kinematics", "Ballistic flight time",           f"{t_air_ballistic*1000:.1f} ms  (measured: {t_air_measured*1000:.1f} ms)"),
    ("",           "",                                ""),
    ("Dynamics",   "Avg launch acceleration",         f"{avg_launch_accel:.1f} m/s²  ({avg_launch_accel/g:.1f} g)"),
    ("Dynamics",   "Avg launch force",                f"{avg_launch_force:.2f} N  ({avg_launch_force/weight_N:.1f}× body weight)"),
    ("Dynamics",   "Impulse",                         f"{impulse_Ns*1000:.2f} N·ms"),
    ("Dynamics",   "KE at liftoff",                   f"{ke_liftoff*1000:.2f} mJ"),
    ("Dynamics",   "PE at peak (measured height)",    f"{pe_peak*1000:.2f} mJ"),
    ("Dynamics",   "Energy loss (KE→PE)",             f"{(ke_liftoff-pe_peak)*1000:.2f} mJ  ({(1-pe_peak/ke_liftoff)*100:.1f}%)"),
]

cat_w = max(len(r[0]) for r in rows) + 2
lab_w = max(len(r[1]) for r in rows) + 2
val_w = max(len(r[2]) for r in rows) + 2
total = cat_w + lab_w + val_w + 4

print()
print("═" * total)
print(f"{'Jump Performance Summary':^{total}}")
print("═" * total)
last_cat = None
for cat, label, value in rows:
    if not label:
        print()
        continue
    if cat and cat != last_cat:
        print(f"  {'─'*4} {cat} {'─'*(total - len(cat) - 9)}")
        last_cat = cat
    print(f"    {label:<{lab_w}}{value}")
print("═" * total)
print()

# ── Jump height on other celestial bodies ─────────────────────────────────────
# h_peak = v0² / (2 * g_body)   (same launch velocity, different gravity)
bodies = [
    ("Earth",    9.807),
    ("Mars",     3.721),
    ("Io",       1.796),
    ("Luna",     1.620),
    ("Titan",    1.352),
    ("Europa",   1.315),
    ("Callisto", 1.235),
    ("Pluto",    0.620),
    ("Ceres",    0.284),
    ("Enceladus", 0.113)
]

bname_w = max(len(b[0]) for b in bodies) + 2
print("─" * 52)
print(f"{'Jump Height on Celestial Bodies':^52}")
print(f"  (launch velocity: {v0:.3f} m/s, mass: {mass_kg*1000:.0f} g)")
print("─" * 52)
print(f"  {'Body':<{bname_w}}  {'g (m/s²)':>10}  {'Height (cm)':>12}  {'× Earth':>8}")
print("─" * 52)
h_earth = v0**2 / (2 * 9.807)
for name, g_body in bodies:
    h_body = v0**2 / (2 * g_body)
    ratio  = h_body / h_earth
    print(f"  {name:<{bname_w}}  {g_body:>10.3f}  {h_body*100:>11.1f}  {ratio:>7.1f}×")
print("─" * 52)
print()

# ── Elastic tube force at full tension ────────────────────────────────────────
# Geometry: open belt around 2 equal cylinders.
#   Path length = 2 × (c-to-c distance)  +  2 × π × r_cyl
#   (The two straight runs each equal the c-to-c distance for equal-radius cylinders;
#    each cylinder contributes one semicircular arc = π × r.)
#
# Force model from Untitled-2/3:
#   T = base_force × (stretch_ratio / ref_ratio)^exponent   [tube tension]
#   Net closing force on cylinders = 2T  (2 parallel straight runs, both pulling inward)
#
# "1 strand" interpretation: the tube has 2 parallel straight runs → counts as 2 strands.
# The closing force per tube = 2T.  T alone = force of 1 half-run (1 strand equivalent).

import math as _math

r_cyl_cm     = 0.8      # cylinder radius (cm)
d_rest_cm    = 6.6      # c-to-c at rest (cm)
d_tens_cm    = 14.0     # c-to-c at full tension (cm)
L_nat_cm     = 21.0     # tube natural (unstretched) length (cm)

def belt_path_cm(d):
    return 2 * d + 2 * _math.pi * r_cyl_cm

L_rest_cm   = belt_path_cm(d_rest_cm)
L_tens_cm   = belt_path_cm(d_tens_cm)
d_taut_cm   = (L_nat_cm - 2 * _math.pi * r_cyl_cm) / 2   # c-to-c where slack disappears

slack_cm        = L_nat_cm - L_rest_cm
extension_cm    = L_tens_cm - L_nat_cm
stretch_ratio_t = L_tens_cm / L_nat_cm

# Force model parameters (from Untitled-3; Untitled-2 uses ref=3.5)
base_kgf  = 5.69   # tube tension at reference stretch ratio (kgf)
ref_ratio = 4.0    # Untitled-3 reference stretch ratio
exponent  = 1.4

T_kgf     = base_kgf * (stretch_ratio_t / ref_ratio) ** exponent
T_N       = T_kgf * 9.81
F_kgf     = 2 * T_kgf   # closing force (2 runs)
F_N       = 2 * T_N

# Also compute with Untitled-2 reference ratio for comparison
T_kgf_35  = base_kgf * (stretch_ratio_t / 3.5) ** exponent
F_N_35    = 2 * T_kgf_35 * 9.81

W = 56
print("─" * W)
print(f"{'Elastic Tube Force at Full Tension':^{W}}")
print("─" * W)
print(f"  {'Cylinder radius':<36} {r_cyl_cm:.1f} cm")
print(f"  {'C-to-C at rest':<36} {d_rest_cm:.1f} cm")
print(f"  {'C-to-C at full tension':<36} {d_tens_cm:.1f} cm")
print(f"  {'Tube natural length':<36} {L_nat_cm:.1f} cm")
print()
print(f"  {'Belt path at rest':<36} {L_rest_cm:.2f} cm")
print(f"  {'Slack at rest (tube - path)':<36} {slack_cm:.2f} cm")
print(f"  {'C-to-C where tube becomes taut':<36} {d_taut_cm:.2f} cm")
print()
print(f"  {'Belt path at full tension':<36} {L_tens_cm:.2f} cm")
print(f"  {'Extension (path - natural length)':<36} {extension_cm:.2f} cm")
print(f"  {'Stretch ratio (L_tens / L_nat)':<36} {stretch_ratio_t:.4f}×")
print()
print(f"  {'Force model (Untitled-3, ref=4.0)':}")
print(f"    Tube tension T            {T_N:>8.2f} N  ({T_kgf:.3f} kgf)  [1 run]")
print(f"    Closing force 2T (1 tube) {F_N:>8.2f} N  ({F_kgf:.3f} kgf)  [2 runs]")
print(f"  {'Comparison (Untitled-2, ref=3.5)':}")
print(f"    Closing force 2T (1 tube) {F_N_35:>8.2f} N  ({F_N_35/9.81:.3f} kgf)")
print()
print(f"  Note: base_force = {base_kgf} kgf/strand is the value used in prior")
print(f"  design calc files; stretch ratio at full tension is only {stretch_ratio_t:.2f}×,")
print(f"  well below the ~4× design point, so force is moderate.")
print("─" * W)
print()

# ── Build trajectory curves ──────────────────────────────────────────────────
# Launch phase: quadratic from h=0 at spring release to h=h_liftoff at liftoff.
# Uses constant average acceleration a = 2*h_liftoff/t_contact^2.
# ── Build trajectory curves ──────────────────────────────────────────────────
# Launch phase: reference point is pinned to the ground (h=0).
# Velocity builds linearly from 0 → v0 over t_contact (constant avg accel).
tau_l  = np.linspace(-t_contact, 0, 200)
h_l    = np.zeros_like(tau_l)
v_l    = avg_launch_accel * (tau_l + t_contact)
a_l    = np.full_like(tau_l, avg_launch_accel / g)

# Free-flight phase: constrained parabola fit, connects at h=0, v=v0 at tau=0.
tau_fl = np.linspace(0, t_air_measured, 500)
h_fl   = np.polyval(p, tau_fl)
v_fl   = np.polyval(np.polyder(p), tau_fl)
a_fl   = np.full_like(tau_fl, -1.0)

t_all  = np.concatenate([tau_l, tau_fl]) * 1000   # ms relative to liftoff
h_all  = np.concatenate([h_l,   h_fl])  * 100     # cm
v_all  = np.concatenate([v_l,   v_fl])             # m/s
a_all  = np.concatenate([a_l,   a_fl])             # g

# Measured points relative to liftoff
tau_meas_all = meas_t - t_liftoff

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
fig.suptitle('Jump Kinematics — Frame-by-Frame Video Analysis', fontsize=13, fontweight='bold')

def vline(ax):
    ax.axvline(0, color='red', linestyle='--', linewidth=0.9, label='Liftoff')
    ax.axhline(0, color='gray', linestyle=':', linewidth=0.6)
    ax.grid(True, alpha=0.3)

# Height
axes[0].plot(t_all, h_all, color='steelblue', linewidth=1.8, label='Fitted curve')
axes[0].scatter(tau_meas_all * 1000, meas_h_cm,
                color='orange', zorder=5, s=40, label='Measured points')
axes[0].set_ylabel('Height (cm)')
axes[0].legend(fontsize=8)
vline(axes[0])

# Velocity
axes[1].plot(t_all, v_all, color='darkorange', linewidth=1.8)
axes[1].set_ylabel('Velocity (m/s)')
vline(axes[1])

# Acceleration
axes[2].plot(t_all, a_all, color='mediumseagreen', linewidth=1.8)
axes[2].axhline(-1, color='gray', linestyle=':', linewidth=0.6)
axes[2].set_ylabel('Acceleration (g)')
axes[2].set_xlabel('Time relative to liftoff (ms)')
axes[2].annotate(f'{avg_launch_accel/g:.1f} g avg launch',
                 xy=(tau_l[100]*1000, avg_launch_accel/g),
                 xytext=(-30, -25), textcoords='offset points',
                 fontsize=8, arrowprops=dict(arrowstyle='->', color='gray'))
axes[2].annotate('−1 g (free fall)',
                 xy=(tau_fl[250]*1000, -1.0),
                 xytext=(15, 10), textcoords='offset points', fontsize=8)
vline(axes[2])

plt.tight_layout()
plt.savefig('jump_kinematics.png', dpi=150)
print("Plot saved to jump_kinematics.png")
plt.show()
