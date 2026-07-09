#!/usr/bin/env python3
"""
Estimate friction (damping) and inertia from Bode plot data.
Fits a first-order model: G(s) = 1 / (I*s + B)
where I = inertia, B = damping coefficient (friction)
"""

import sys
import numpy as np
import pandas as pd
from scipy import signal, optimize
import os
import matplotlib.pyplot as plt

def estimate_damped_system(csv_file, nperseg=1024):
    """
    Fit a damped system model: G(s) = 1 / (I*s + B)
    
    In frequency domain:
    G(jω) = 1 / (B + j*I*ω)
    |G(jω)| = 1 / sqrt(B² + (I*ω)²)
    
    Units:
    - B (damping): Nm⋅s/rad
    - I (inertia): kg⋅m²
    - Velocity: deg/s → rad/s
    - Torque: Nm
    """
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        return None
    
    # Load data
    df = pd.read_csv(csv_file)
    print(f"Loaded: {csv_file}\n")
    
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
    print(f"Samples: {len(torque)}\n")
    
    # Detrend
    torque_detrended = signal.detrend(torque)
    velocity_detrended = signal.detrend(velocity_rad_s)
    
    # Compute transfer function via spectral analysis
    f, Pxy = signal.csd(torque_detrended, velocity_detrended, fs=fs, nperseg=nperseg)
    f, Pxx = signal.welch(torque_detrended, fs=fs, nperseg=nperseg)
    
    Pxx = np.maximum(Pxx, 1e-10)
    H = Pxy / Pxx  # Transfer function
    
    magnitude = np.abs(H)
    phase = np.angle(H)
    omega = 2 * np.pi * f  # rad/s
    
    # =========================================================================
    # FIT MODEL: |G(jω)| = 1 / sqrt(B² + (I*ω)²)
    # =========================================================================
    print("="*70)
    print("FITTING DAMPED SYSTEM MODEL: G(s) = 1 / (I*s + B)")
    print("="*70)
    
    # Use frequencies from 0.5 Hz to 50 Hz for fitting
    freq_mask = (f >= 0.5) & (f <= 50) & (magnitude > 1e-6)
    f_fit = f[freq_mask]
    mag_fit = magnitude[freq_mask]
    omega_fit = 2 * np.pi * f_fit
    
    print(f"Fitting using {len(f_fit)} frequency points: {f_fit[0]:.2f} - {f_fit[-1]:.2f} Hz\n")
    
    # Define model and cost function
    def model(params, omega):
        """Transfer function magnitude: 1 / sqrt(B² + (I*ω)²)"""
        I, B = params
        if I <= 0 or B <= 0:
            return np.ones_like(omega) * 1e10
        return 1.0 / np.sqrt(B**2 + (I * omega)**2)
    
    def residual(params, omega, magnitude):
        """Residual in dB scale (more stable)"""
        mag_model = model(params, omega)
        mag_db_measured = 20 * np.log10(magnitude + 1e-10)
        mag_db_model = 20 * np.log10(mag_model + 1e-10)
        return mag_db_model - mag_db_measured
    
    # Initial guess
    I_init = 0.06  # kg⋅m²
    B_init = 0.1   # Nm⋅s/rad
    
    # Fit using Levenberg-Marquardt
    try:
        result = optimize.least_squares(
            residual, [I_init, B_init],
            args=(omega_fit, mag_fit),
            bounds=([1e-6, 1e-6], [10, 10]),
            max_nfev=1000
        )
        
        I_fit, B_fit = result.x
        residual_rms = np.sqrt(np.mean(result.fun**2))
        
        print(f"✓ Fit converged (RMS error: {residual_rms:.4f} dB)\n")
        
    except Exception as e:
        print(f"✗ Fit failed: {e}\n")
        return None
    
    # =========================================================================
    # RESULTS
    # =========================================================================
    print("="*70)
    print("ESTIMATED SYSTEM PARAMETERS")
    print("="*70)
    print(f"Inertia (I):        {I_fit:.6f} kg⋅m²")
    print(f"Damping (B):        {B_fit:.6f} Nm⋅s/rad")
    print(f"Fit RMS error:      {residual_rms:.4f} dB\n")
    
    # Calculate derived quantities
    natural_freq = 1.0 / (2 * np.pi) * np.sqrt(0)  # Not applicable for first-order
    damping_ratio_at_cutoff = B_fit / (2 * np.sqrt(I_fit * 0.001))  # Rough estimate
    
    # Characteristic frequency (corner frequency where inertia and damping equal)
    # B = I * ω_c  →  ω_c = B / I
    omega_c = B_fit / I_fit
    f_c = omega_c / (2 * np.pi)
    
    print("DERIVED QUANTITIES")
    print("="*70)
    print(f"Corner frequency:   {f_c:.2f} Hz (where B = I*ω)")
    print(f"  (below this: friction-dominated; above: inertia-dominated)\n")
    
    # Low-frequency gain (friction only): G(0) = 1/B
    gain_dc = 1.0 / B_fit
    print(f"DC gain (ω→0):      {gain_dc:.6f} (rad/s)/Nm")
    print(f"  (velocity/torque with no acceleration, pure friction effect)\n")
    
    # High-frequency behavior: G(∞) ~ 1/(I*ω)
    # At 10 Hz:
    omega_10hz = 2 * np.pi * 10
    gain_10hz_theory = 1.0 / (I_fit * omega_10hz)
    gain_10hz_model = model([I_fit, B_fit], omega_10hz)
    print(f"Gain at 10 Hz:")
    print(f"  Model:              {gain_10hz_model:.6f} (rad/s)/Nm")
    print(f"  High-freq approx:   {gain_10hz_theory:.6f} (rad/s)/Nm")
    print(f"  (deviation shows friction effect)\n")
    
    # =========================================================================
    # FRICTION TORQUE
    # =========================================================================
    print("="*70)
    print("FRICTION/VISCOUS DAMPING")
    print("="*70)
    
    # Viscous friction: τ_friction = B * ω
    # At different velocities (in rad/s):
    velocities_rad_s = np.array([1.0, 5.0, 10.0, 20.0, 50.0])
    
    print("\nViscous friction torque = B × velocity:")
    print(f"{'Velocity (rad/s)':<20} {'Friction Torque (Nm)':<25}")
    print("-" * 45)
    for vel in velocities_rad_s:
        tau_friction = B_fit * vel
        print(f"{vel:<20.1f} {tau_friction:<25.4f}")
    
    # Also in deg/s
    velocities_deg_s = velocities_rad_s * 180.0 / np.pi
    print(f"\nOr in deg/s:")
    print(f"{'Velocity (deg/s)':<20} {'Friction Torque (Nm)':<25}")
    print("-" * 45)
    for vel_deg, vel_rad in zip(velocities_deg_s, velocities_rad_s):
        tau_friction = B_fit * vel_rad
        print(f"{vel_deg:<20.1f} {tau_friction:<25.4f}")
    
    print(f"\n✓ Friction coefficient B = {B_fit:.6f} Nm⋅s/rad")
    print(f"  At 1 rad/s (57.3 deg/s): {B_fit:.4f} Nm friction torque")
    
    # =========================================================================
    # PLOT COMPARISON
    # =========================================================================
    print("\n" + "="*70)
    print("GENERATING COMPARISON PLOT...")
    print("="*70 + "\n")
    
    # Frequency range for plotting
    f_plot = f[(f > 0.5) & (f < 200)]
    omega_plot = 2 * np.pi * f_plot
    mag_measured = magnitude[(f > 0.5) & (f < 200)]
    mag_model_plot = model([I_fit, B_fit], omega_plot)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Magnitude plot
    ax1.loglog(f_plot, mag_measured, 'b-', linewidth=2, label='Measured')
    ax1.loglog(f_plot, mag_model_plot, 'r--', linewidth=2, label=f'Fit: I={I_fit:.4f}, B={B_fit:.4f}')
    ax1.axvline(f_c, color='g', linestyle=':', alpha=0.5, label=f'Corner: {f_c:.2f} Hz')
    ax1.grid(True, which='both', alpha=0.3)
    ax1.set_ylabel('Magnitude (rad/s)/Nm', fontsize=12)
    ax1.set_title('System Model Fit: G(s) = 1/(I*s + B)', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    
    # Error plot
    mag_db_measured = 20 * np.log10(mag_measured + 1e-10)
    mag_db_model = 20 * np.log10(mag_model_plot + 1e-10)
    error_db = mag_db_model - mag_db_measured
    
    ax2.semilogx(f_plot, error_db, 'k-', linewidth=1.5)
    ax2.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax2.fill_between(f_plot, -1, 1, alpha=0.2, color='green', label='±1 dB')
    ax2.grid(True, which='both', alpha=0.3)
    ax2.set_xlabel('Frequency (Hz)', fontsize=12)
    ax2.set_ylabel('Error (dB)', fontsize=12)
    ax2.set_ylim([-5, 5])
    ax2.legend()
    
    plt.tight_layout()
    output_file = os.path.splitext(csv_file)[0] + '_fit.png'
    plt.savefig(output_file, dpi=150)
    print(f"✅ Fit plot saved: {output_file}\n")
    plt.show()
    
    return {
        'motor_id': motor_id,
        'inertia': I_fit,
        'damping': B_fit,
        'corner_freq': f_c,
        'dc_gain': gain_dc,
        'rms_error': residual_rms,
    }

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 estimate_damping.py <aligned_csv_file>")
        print("Example: python3 estimate_damping.py motor_log_aligned_20260608_170004.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    estimate_damped_system(csv_file)
