#!/usr/bin/env python3
"""
Bode plot generation from aligned torque/velocity CSV data.
Usage:
    python3 plot_bode.py <csv_file>
    python3 plot_bode.py motor_log_aligned_20260608_170004.csv
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
import os

def load_aligned_csv(filepath):
    """Load aligned CSV and extract torque and velocity signals."""
    if not os.path.exists(filepath):
        print(f"Error: File not found: {filepath}")
        sys.exit(1)
    
    df = pd.read_csv(filepath)
    print(f"Loaded CSV: {filepath}")
    print(f"Columns: {df.columns.tolist()}")
    print(f"Shape: {df.shape}")
    
    return df

def extract_signals(df):
    """Extract torque (input) and velocity (output) signals from aligned CSV."""
    # Find motor columns
    torque_cols = [col for col in df.columns if 'actual_torque_m' in col]
    vel_cols = [col for col in df.columns if 'vel_deg_s_m' in col]
    
    if not torque_cols or not vel_cols:
        print("Error: Could not find torque or velocity columns")
        print(f"Available columns: {df.columns.tolist()}")
        sys.exit(1)
    
    # Use first motor if multiple
    torque_col = torque_cols[0]
    vel_col = vel_cols[0]
    
    motor_id = torque_col.split('_m')[-1]
    print(f"\nUsing motor {motor_id}")
    print(f"Torque column: {torque_col}")
    print(f"Velocity column: {vel_col}")
    
    # Extract signals
    timestamp_ms = df['timestamp_ms'].values
    torque = df[torque_col].values
    velocity = df[vel_col].values
    
    # Remove NaN values
    valid_mask = ~(np.isnan(torque) | np.isnan(velocity))
    timestamp_ms = timestamp_ms[valid_mask]
    torque = torque[valid_mask]
    velocity = velocity[valid_mask]
    
    print(f"\nSignal lengths: {len(torque)} samples")
    print(f"Torque range: [{np.min(torque):.3f}, {np.max(torque):.3f}] Nm")
    print(f"Velocity range: [{np.min(velocity):.3f}, {np.max(velocity):.3f}] deg/s")
    
    # Calculate sampling period
    dt_ms = np.median(np.diff(timestamp_ms))
    dt_s = dt_ms / 1000.0
    fs = 1.0 / dt_s  # sampling frequency
    
    print(f"Median sample interval: {dt_ms:.2f} ms")
    print(f"Sampling frequency: {fs:.2f} Hz")
    
    return torque, velocity, fs

def compute_bode(torque, velocity, fs, nperseg=1024):
    """
    Compute Bode plot from torque (input) and velocity (output).
    
    Transfer function: G(f) = V(f) / T(f)
    - Magnitude: |G(f)| = |V(f)| / |T(f)|
    - Phase: ∠G(f) = ∠V(f) - ∠T(f)
    """
    
    # Detrend signals
    torque_detrended = signal.detrend(torque)
    velocity_detrended = signal.detrend(velocity)
    
    # Compute cross-spectral density and auto-spectral densities
    f, Pxy = signal.csd(torque_detrended, velocity_detrended, fs=fs, nperseg=nperseg)
    f, Pxx = signal.welch(torque_detrended, fs=fs, nperseg=nperseg)
    f, Pyy = signal.welch(velocity_detrended, fs=fs, nperseg=nperseg)
    
    # Compute transfer function H(f) = Pxy(f) / Pxx(f)
    # Avoid division by zero
    Pxx = np.maximum(Pxx, 1e-10)
    H = Pxy / Pxx
    
    # Magnitude in dB (deg/s per Nm)
    magnitude_db = 20 * np.log10(np.abs(H) + 1e-10)
    
    # Phase in degrees
    phase_deg = np.angle(H, deg=True)
    
    # Unwrap phase for smoother plot
    phase_deg_unwrapped = np.unwrap(phase_deg * np.pi / 180.0, discont=np.pi) * 180.0 / np.pi
    
    return f, magnitude_db, phase_deg_unwrapped, H

def plot_bode(f, magnitude_db, phase_deg, output_file=None):
    """Plot Bode diagram with magnitude and phase."""
    
    # Filter to meaningful frequency range (exclude DC and very high freq noise)
    valid_mask = (f > 0.5) & (f < 200)  # 0.5 Hz to 200 Hz
    f_plot = f[valid_mask]
    mag_plot = magnitude_db[valid_mask]
    phase_plot = phase_deg[valid_mask]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Magnitude plot
    ax1.semilogx(f_plot, mag_plot, 'b-', linewidth=2)
    ax1.grid(True, which='both', alpha=0.3)
    ax1.set_ylabel('Magnitude (dB)', fontsize=12)
    ax1.set_title('Bode Plot: Transfer Function G(f) = Velocity / Torque', fontsize=14, fontweight='bold')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Add -3dB line for reference
    ax1.axhline(y=-3, color='r', linestyle='--', alpha=0.3, label='-3dB')
    ax1.legend()
    
    # Phase plot
    ax2.semilogx(f_plot, phase_plot, 'r-', linewidth=2)
    ax2.grid(True, which='both', alpha=0.3)
    ax2.set_xlabel('Frequency (Hz)', fontsize=12)
    ax2.set_ylabel('Phase (degrees)', fontsize=12)
    ax2.axhline(y=-90, color='k', linestyle='--', alpha=0.3)
    ax2.axhline(y=-45, color='g', linestyle='--', alpha=0.3, label='-45°')
    ax2.axhline(y=-180, color='k', linestyle='--', alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150)
        print(f"\n✅ Bode plot saved: {output_file}")
    
    plt.show()
    
    return fig

def print_transfer_function_info(f, H, magnitude_db):
    """Print key characteristics of the transfer function."""
    print("\n" + "="*60)
    print("TRANSFER FUNCTION CHARACTERISTICS")
    print("="*60)
    
    # Find peak magnitude and frequency
    valid_mask = (f > 0.5) & (f < 200)
    peak_idx = np.argmax(magnitude_db[valid_mask])
    peak_freq = f[valid_mask][peak_idx]
    peak_mag = magnitude_db[valid_mask][peak_idx]
    
    print(f"Peak gain: {peak_mag:.2f} dB ({10**(peak_mag/20):.4f} (deg/s)/Nm)")
    print(f"Peak frequency: {peak_freq:.2f} Hz")
    
    # Find -3dB bandwidth
    max_mag = np.max(magnitude_db[valid_mask])
    threshold = max_mag - 3
    above_threshold = magnitude_db[valid_mask] >= threshold
    if np.any(above_threshold):
        freq_above = f[valid_mask][above_threshold]
        bw_low = np.min(freq_above)
        bw_high = np.max(freq_above)
        bw = bw_high - bw_low
        print(f"-3dB Bandwidth: {bw_low:.2f} Hz to {bw_high:.2f} Hz (span: {bw:.2f} Hz)")
    
    # DC gain (at lowest frequency)
    dc_gain_idx = np.argmin(np.abs(f[valid_mask] - f[valid_mask][0]))
    dc_gain_db = magnitude_db[valid_mask][dc_gain_idx]
    print(f"Low-freq gain: {dc_gain_db:.2f} dB")
    
    print("="*60)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_bode.py <aligned_csv_file>")
        print("Example: python3 plot_bode.py motor_log_aligned_20260608_170004.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    # Load data
    df = load_aligned_csv(csv_file)
    
    # Extract signals
    torque, velocity, fs = extract_signals(df)
    
    # Compute Bode plot
    print("\nComputing FFT and transfer function...")
    f, magnitude_db, phase_deg, H = compute_bode(torque, velocity, fs, nperseg=1024)
    
    # Print characteristics
    print_transfer_function_info(f, H, magnitude_db)
    
    # Generate output filename
    base_name = os.path.splitext(csv_file)[0]
    output_file = f"{base_name}_bode.png"
    
    # Plot and save
    print("\nGenerating Bode plot...")
    plot_bode(f, magnitude_db, phase_deg, output_file=output_file)

if __name__ == '__main__':
    main()
