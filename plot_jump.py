"""
plot_jump.py — reads jump acceleration data from Arduino and plots it.

Usage:
  1. Set COM_PORT to your Arduino's port (e.g. 'COM3' on Windows).
  2. Run this script, then press 'd' in the Arduino serial monitor (or type here).
     The script will collect 4 s of accel data and plot it automatically.
"""

import serial
import matplotlib.pyplot as plt
import numpy as np
import time

COM_PORT     = '/dev/ttyACM0'   # <-- change to your port
BAUD         = 115200
CLIP_RAW     = 32700    # raw counts above this are saturated (max int16 = 32767)
SMOOTH_HALF  = 3        # moving average half-window: ±3 → 7-sample window

def nan_moving_avg(arr, half_w):
    """Centered moving average that skips NaN (clipped) values."""
    result = np.empty_like(arr)
    n = len(arr)
    for i in range(n):
        window = arr[max(0, i - half_w):min(n, i + half_w + 1)]
        valid  = window[~np.isnan(window)]
        result[i] = np.mean(valid) if len(valid) > 0 else np.nan
    return result

def collect_jump_data(ser: serial.Serial):
    t, ax, ay, az = [], [], [], []
    print("Waiting for jump trigger ('d' key sent to Arduino)...")
    print("Send 'd' via the Arduino Serial Monitor, or press Enter here to send it now.")

    # Optional: send 'd' from this script
    input()
    ser.write(b'd')

    # Wait for CSV header line
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('t_ms'):
            print("Recording started — collecting 2 s of data...")
            break

    # Read rows until "# recording complete"
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line or line.startswith('#'):
            print(line if line else "")
            ser.write(b's')
            break
        parts = line.split(',')
        if len(parts) == 4:
            try:
                t.append(int(parts[0]))
                ax.append(int(parts[1]))
                ay.append(int(parts[2]))
                az.append(int(parts[3]))
            except ValueError:
                pass  # skip malformed lines

    return t, ax, ay, az


def save_csv(t, ax_g, ay_g, az_g, ax_sm, ay_sm, az_sm):
    """Save raw and smoothed acceleration data to CSV."""
    t_s = np.array(t) / 1000.0
    header = 't_s,ax_raw_g,ay_raw_g,az_raw_g,ax_smooth_g,ay_smooth_g,az_smooth_g'
    rows = np.column_stack([t_s, ax_g, ay_g, az_g, ax_sm, ay_sm, az_sm])
    np.savetxt('jump_accel.csv', rows, delimiter=',', header=header, comments='', fmt='%.6f')
    print("CSV saved to jump_accel.csv")


def plot(t, ax, ay, az):
    # Convert raw LSB to g (±16g range → 2048 LSB/g)
    scale = 1 / 2048.0
    t_s  = np.array(t) / 1000.0
    axes_raw = [
        np.array(ax) * scale,
        np.array(ay) * scale,
        np.array(az) * scale,
    ]

    # Mark clipped samples as NaN
    clip_threshold = CLIP_RAW * scale
    axes_clean = []
    for ch in axes_raw:
        c = ch.copy()
        c[np.abs(c) >= clip_threshold] = np.nan
        axes_clean.append(c)

    # NaN-aware moving average
    axes_smooth = [nan_moving_avg(c, SMOOTH_HALF) for c in axes_clean]

    labels = ['X', 'Y', 'Z']
    colors = ['steelblue', 'darkorange', 'mediumseagreen']

    # --- Unsmoothed plot ---
    fig1, ax_plot1 = plt.subplots(figsize=(10, 5))
    for raw, label, color in zip(axes_raw, labels, colors):
        ax_plot1.plot(t_s, raw, color=color, linewidth=1.0, label=label)
    any_clipped = np.zeros(len(t_s), dtype=bool)
    for ch in axes_raw:
        any_clipped |= np.abs(ch) >= clip_threshold
    if any_clipped.any():
        ax_plot1.fill_between(t_s, -16, 16, where=any_clipped,
                              color='red', alpha=0.12, label='Clipped region')
    ax_plot1.axvline(0, color='red', linestyle='--', linewidth=0.9, label='Jump trigger')
    ax_plot1.set_xlabel('Time (s)')
    ax_plot1.set_ylabel('Acceleration (g)')
    ax_plot1.set_title('Jump Acceleration — MPU6050  (raw)')
    ax_plot1.legend()
    ax_plot1.grid(True, alpha=0.3)
    plt.tight_layout()
    fig1.savefig('jump_accel_raw.png', dpi=150)
    print("Plot saved to jump_accel_raw.png")

    # --- Smoothed plot ---
    fig2, ax_plot2 = plt.subplots(figsize=(10, 5))
    for raw, smooth, label, color in zip(axes_raw, axes_smooth, labels, colors):
        ax_plot2.plot(t_s, raw,    color=color, alpha=0.2, linewidth=0.8)
        ax_plot2.plot(t_s, smooth, color=color, linewidth=1.8, label=label)
    if any_clipped.any():
        ax_plot2.fill_between(t_s, -16, 16, where=any_clipped,
                              color='red', alpha=0.12, label='Clipped region')
    ax_plot2.axvline(0, color='red', linestyle='--', linewidth=0.9, label='Jump trigger')
    ax_plot2.set_xlabel('Time (s)')
    ax_plot2.set_ylabel('Acceleration (g)')
    ax_plot2.set_title(f'Jump Acceleration — MPU6050  (smoothed ±{SMOOTH_HALF} samples, clipped removed)')
    ax_plot2.legend()
    ax_plot2.grid(True, alpha=0.3)
    plt.tight_layout()
    fig2.savefig('jump_accel_smooth.png', dpi=150)
    print("Plot saved to jump_accel_smooth.png")

    # --- CSV ---
    save_csv(t, axes_raw[0], axes_raw[1], axes_raw[2],
             axes_smooth[0], axes_smooth[1], axes_smooth[2])

    plt.show()


if __name__ == '__main__':
    with serial.Serial(COM_PORT, BAUD, timeout=10) as ser:
        time.sleep(2)  # wait for Arduino reset after serial open
        ser.reset_input_buffer()
        t, ax, ay, az = collect_jump_data(ser)

    if t:
        print(f"Collected {len(t)} samples.")
        scale = 1 / 2048.0
        clip_threshold = CLIP_RAW * scale
        n_clipped = sum(1 for v in ax + ay + az if abs(v) * scale >= clip_threshold)
        print(f"Clipped samples: {n_clipped}")
        peak_ax = max(abs(v) * scale for v in ax)
        peak_ay = max(abs(v) * scale for v in ay)
        peak_az = max(abs(v) * scale for v in az)
        print(f"Peak |X|: {peak_ax:.3f} g")
        print(f"Peak |Y|: {peak_ay:.3f} g")
        print(f"Peak |Z|: {peak_az:.3f} g")
        plot(t, ax, ay, az)
    else:
        print("No data received.")

