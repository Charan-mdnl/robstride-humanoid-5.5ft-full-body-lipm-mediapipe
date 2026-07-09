#!/usr/bin/env python3
"""
Estimate inertia from Bode plot data (Transfer Function: Velocity / Torque)
Usage:
    python3 estimate_inertia.py <csv_file>
"""

import sys
import numpy as np
import pandas as pd
from scipy import signal
import os

def estimate_inertia_from_csv(csv_file, nperseg=1024):
    """
    Estimate moment of inertia from torque/velocity transfer function.
    
    Theory:
    - Torque = I × Angular_Acceleration
    - In frequency domain: G(s) = Velocity(s) / Torque(s) = 1 / (I × s)
    - Magnitude: |G(jω)| = 1 / (I × 2πf)
    - Therefore: I = 1 / (2πf × |G(f)|)
    
    Units conversion:
    - Velocity: deg/s → rad/s (multiply by π/180)
    - Torque: Nm
    - Inertia: kg⋅m²
    """
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        return None
    
    # Load data
    df = pd.read_csv(csv_file)
    print(f"Loaded: {csv_file}")
    
    # Find motor columns
    torque_cols = [col for col in df.columns if 'actual_torque_m' in col]
    vel_cols = [col for col in df.columns if 'vel_deg_s_m' in col]
    
    if not torque_cols or not vel_cols:
        print(f"Error: Could not find torque/velocity columns")
        return None
    
    torque_col = torque_cols[0]
    vel_col = vel_cols[0]
    motor_id = torque_col.split('_m')[-1]
    
    # Extract signals
    timestamp_ms = df['timestamp_ms'].values
    torque = df[torque_col].values
    velocity_deg_s = df[vel_col].values
    
    # Remove NaN
    valid_mask = ~(np.isnan(torque) | np.isnan(velocity_deg_s))
    timestamp_ms = timestamp_ms[valid_mask]
    torque = torque[valid_mask]
    velocity_deg_s = velocity_deg_s[valid_mask]
    
    # Convert velocity from deg/s to rad/s
    velocity_rad_s = velocity_deg_s * np.pi / 180.0
    
    # Sampling frequency
    dt_ms = np.median(np.diff(timestamp_ms))
    fs = 1000.0 / dt_ms  # Hz
    print(f"Motor: {motor_id}, Sampling rate: {fs:.2f} Hz")
    print(f"Samples: {len(torque)}")
    
    # Detrend
    torque_detrended = signal.detrend(torque)
    velocity_detrended = signal.detrend(velocity_rad_s)
    
    # Compute transfer function via spectral analysis
    f, Pxy = signal.csd(torque_detrended, velocity_detrended, fs=fs, nperseg=nperseg)
    f, Pxx = signal.welch(torque_detrended, fs=fs, nperseg=nperseg)
    
    Pxx = np.maximum(Pxx, 1e-10)
    H = Pxy / Pxx  # Transfer function
    
    magnitude = np.abs(H)
    omega = 2 * np.pi * f  # rad/s
    
    # =========================================================================
    # METHOD 1: Estimate inertia from low-frequency region (< 5 Hz)
    # =========================================================================
    print("\n" + "="*70)
    print("METHOD 1: Low-Frequency Inertia Estimate")
    print("="*70)
    
    # Use frequencies between 1 Hz and 5 Hz where system is dominated by inertia
    freq_mask = (f >= 1.0) & (f <= 5.0)
    f_low = f[freq_mask]
    mag_low = magnitude[freq_mask]
    omega_low = omega[freq_mask]
    
    if len(f_low) > 0:
        # I = 1 / (2πf × |G(f)|)
        I_estimates = 1.0 / (omega_low * mag_low)
        I_mean = np.mean(I_estimates)
        I_std = np.std(I_estimates)
        
        print(f"Frequency range: {f_low[0]:.2f} - {f_low[-1]:.2f} Hz")
        print(f"Inertia estimate: {I_mean:.6f} ± {I_std:.6f} kg⋅m²")
        print(f"Inertia range: {np.min(I_estimates):.6f} to {np.max(I_estimates):.6f} kg⋅m²")
    
    # =========================================================================
    # METHOD 2: Fit power law at low frequencies
    # =========================================================================
    print("\n" + "="*70)
    print("METHOD 2: Power-Law Fit (Low-Frequency Region)")
    print("="*70)
    
    # Fit: |G(f)| = K / f  →  log(|G|) = log(K) - log(f)
    # For pure inertia: K = 1 / (2π × I)
    
    freq_mask_fit = (f >= 0.5) & (f <= 10.0) & (magnitude > 1e-5)
    f_fit = f[freq_mask_fit]
    mag_fit = magnitude[freq_mask_fit]
    
    if len(f_fit) > 3:
        # Log-log fit
        log_f = np.log(f_fit)
        log_mag = np.log(mag_fit)
        
        # Linear regression: log(mag) = a + b*log(f)
        coeffs = np.polyfit(log_f, log_mag, 1)
        slope = coeffs[0]
        intercept = coeffs[1]
        
        print(f"Log-log fit: log(|G|) = {intercept:.4f} + {slope:.4f}×log(f)")
        print(f"Slope: {slope:.4f} (ideal: -1.0 for pure inertia)")
        
        # Extract inertia: K = exp(intercept) = 1 / (2π × I)
        K = np.exp(intercept)
        I_fit = 1.0 / (2 * np.pi * K)
        
        print(f"Gain constant K: {K:.6e}")
        print(f"Estimated Inertia: {I_fit:.6f} kg⋅m²")
    
    # =========================================================================
    # METHOD 3: Use the peak frequency and phase
    # =========================================================================
    print("\n" + "="*70)
    print("METHOD 3: Peak-Based Estimate")
    print("="*70)
    
    valid_freq_mask = (f > 0.5) & (f < 200)
    f_valid = f[valid_freq_mask]
    mag_valid = magnitude[valid_freq_mask]
    peak_idx = np.argmax(mag_valid)
    peak_freq = f_valid[peak_idx]
    peak_mag = mag_valid[peak_idx]
    
    # At low frequencies (before damping), approximate I from peak
    I_peak = 1.0 / (2 * np.pi * peak_freq * peak_mag)
    
    print(f"Peak frequency: {peak_freq:.2f} Hz")
    print(f"Peak magnitude: {peak_mag:.6f} (rad/s)/Nm")
    print(f"Inertia estimate (from peak): {I_peak:.6f} kg⋅m²")
    
    # =========================================================================
    # SUMMARY
    # =========================================================================
    print("\n" + "="*70)
    print("SUMMARY - INERTIA ESTIMATES")
    print("="*70)
    
    if len(f_low) > 0:
        I_summary = I_mean
        print(f"Best estimate (low-freq avg): I ≈ {I_summary:.6f} kg⋅m²")
    
    if len(f_fit) > 3:
        print(f"Power-law fit:                I ≈ {I_fit:.6f} kg⋅m²")
    
    print(f"Peak-based estimate:          I ≈ {I_peak:.6f} kg⋅m²")
    
    print("\nNote:")
    print("- These are rough estimates from the frequency response")
    print("- Accuracy depends on frequency resolution and SNR")
    print("- Better estimates require fitting a more complex model")
    print("  that includes damping and stiffness")
    print("="*70)
    
    return {
        'motor_id': motor_id,
        'inertia_low_freq': I_mean if len(f_low) > 0 else None,
        'inertia_fit': I_fit if len(f_fit) > 3 else None,
        'inertia_peak': I_peak,
        'peak_freq': peak_freq,
    }

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 estimate_inertia.py <aligned_csv_file>")
        print("Example: python3 estimate_inertia.py motor_log_aligned_20260608_170004.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    estimate_inertia_from_csv(csv_file)
