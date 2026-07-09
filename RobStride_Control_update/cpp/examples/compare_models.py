#!/usr/bin/env python3
"""
Fit both first-order and second-order damped system models.

First-order (velocity output):
  G(s) = 1 / (I*s + B)
  
Second-order (position output):
  G(s) = 1 / (I*s² + B*s + K)
  or equivalently (for velocity):
  G_v(s) = s / (I*s² + B*s + K)
"""

import sys
import numpy as np
import pandas as pd
from scipy import signal, optimize
import os
import matplotlib.pyplot as plt

def fit_models(csv_file, nperseg=1024):
    """Compare first-order vs second-order system fits."""
    
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
    fs = 1000.0 / dt_ms
    print(f"Motor: {motor_id}, Sampling rate: {fs:.2f} Hz")
    print(f"Samples: {len(torque)}\n")
    
    # Detrend
    torque_detrended = signal.detrend(torque)
    velocity_detrended = signal.detrend(velocity_rad_s)
    
    # Compute transfer function
    f, Pxy = signal.csd(torque_detrended, velocity_detrended, fs=fs, nperseg=nperseg)
    f, Pxx = signal.welch(torque_detrended, fs=fs, nperseg=nperseg)
    
    Pxx = np.maximum(Pxx, 1e-10)
    H = Pxy / Pxx  # G(jω) = V(ω) / T(ω)
    
    magnitude = np.abs(H)
    omega = 2 * np.pi * f
    
    # Fit range
    freq_mask = (f >= 0.5) & (f <= 50) & (magnitude > 1e-6)
    f_fit = f[freq_mask]
    omega_fit = 2 * np.pi * f_fit
    mag_fit = magnitude[freq_mask]
    
    print("="*80)
    print("MODEL 1: FIRST-ORDER (Velocity output)")
    print("="*80)
    print("G(s) = 1 / (I*s + B)\n")
    
    def model1(params, omega):
        I, B = params
        if I <= 0 or B <= 0:
            return np.ones_like(omega) * 1e10
        return 1.0 / np.sqrt(B**2 + (I * omega)**2)
    
    def residual1(params, omega, magnitude):
        mag_model = model1(params, omega)
        mag_db_measured = 20 * np.log10(magnitude + 1e-10)
        mag_db_model = 20 * np.log10(mag_model + 1e-10)
        return mag_db_model - mag_db_measured
    
    try:
        result1 = optimize.least_squares(
            residual1, [0.06, 0.1],
            args=(omega_fit, mag_fit),
            bounds=([1e-6, 1e-6], [10, 10]),
            max_nfev=1000
        )
        I1, B1 = result1.x
        error1 = np.sqrt(np.mean(result1.fun**2))
        print(f"✓ Converged | RMS error: {error1:.4f} dB")
        print(f"  I = {I1:.6f} kg⋅m²")
        print(f"  B = {B1:.6f} Nm⋅s/rad\n")
    except:
        print("✗ Failed to fit\n")
        return None
    
    # =========================================================================
    print("="*80)
    print("MODEL 2: SECOND-ORDER (with stiffness)")
    print("="*80)
    print("G_v(s) = s / (I*s² + B*s + K)")
    print("(position output: G(s) = 1 / (I*s² + B*s + K))\n")
    
    def model2(params, omega):
        """
        For velocity output G_v(s) = s / (I*s² + B*s + K)
        G_v(jω) = jω / (K + jB*ω - I*ω²)
        |G_v(jω)| = ω / sqrt(K² + (B*ω - I*ω²)²)
        """
        I, B, K = params
        if I <= 0 or B <= 0 or K < 0:
            return np.ones_like(omega) * 1e10
        
        numerator = omega
        denominator = np.sqrt(K**2 + (B * omega - I * omega**2)**2)
        return numerator / (denominator + 1e-10)
    
    def residual2(params, omega, magnitude):
        mag_model = model2(params, omega)
        mag_db_measured = 20 * np.log10(magnitude + 1e-10)
        mag_db_model = 20 * np.log10(mag_model + 1e-10)
        return mag_db_model - mag_db_measured
    
    try:
        result2 = optimize.least_squares(
            residual2, [0.06, 0.1, 0.05],  # [I, B, K]
            args=(omega_fit, mag_fit),
            bounds=([1e-6, 1e-6, 0], [10, 10, 10]),
            max_nfev=1000
        )
        I2, B2, K2 = result2.x
        error2 = np.sqrt(np.mean(result2.fun**2))
        print(f"✓ Converged | RMS error: {error2:.4f} dB")
        print(f"  I = {I2:.6f} kg⋅m²")
        print(f"  B = {B2:.6f} Nm⋅s/rad")
        print(f"  K = {K2:.6f} Nm/rad (stiffness)\n")
    except Exception as e:
        print(f"✗ Failed: {e}\n")
        I2, B2, K2 = None, None, None
        error2 = float('inf')
    
    # =========================================================================
    print("="*80)
    print("MODEL COMPARISON")
    print("="*80)
    print(f"{'Model':<30} {'RMS Error (dB)':<20} {'Improvement':<20}")
    print("-" * 70)
    print(f"{'First-order':<30} {error1:<20.4f} {'—':<20}")
    if I2 is not None:
        improvement = error1 - error2
        pct = 100 * improvement / error1 if error1 > 0 else 0
        print(f"{'Second-order':<30} {error2:<20.4f} {f'{improvement:.4f} dB ({pct:.1f}%)':<20}")
    print()
    
    # =========================================================================
    print("="*80)
    print("PHYSICAL INTERPRETATION")
    print("="*80)
    
    if I2 is not None and K2 > 1e-6:
        print(f"\n✓ Stiffness detected! System includes spring-like element.")
        print(f"\n  Natural frequency (undamped): {np.sqrt(K2/I2)/(2*np.pi):.2f} Hz")
        print(f"  Damping ratio: {B2/(2*np.sqrt(I2*K2)):.3f}")
        print(f"  Stiffness torque per radian: {K2:.6f} Nm/rad")
        
        # Which model is better?
        if error2 < error1 - 0.5:  # More than 0.5 dB improvement
            print(f"\n  → Second-order model is significantly better ({improvement:.2f} dB improvement)")
            print(f"  → System has joint/spring stiffness")
            better_model = 2
        else:
            print(f"\n  → First-order model sufficient (improvement only {improvement:.2f} dB)")
            print(f"  → Stiffness is negligible")
            better_model = 1
    else:
        print(f"\n  First-order model is sufficient (no significant stiffness)")
        better_model = 1
    
    # =========================================================================
    print("\n" + "="*80)
    print("GENERATING COMPARISON PLOTS")
    print("="*80 + "\n")
    
    # Plot range
    f_plot = f[(f > 0.5) & (f < 200)]
    omega_plot = 2 * np.pi * f_plot
    mag_measured = magnitude[(f > 0.5) & (f < 200)]
    
    mag_model1 = model1([I1, B1], omega_plot)
    if I2 is not None:
        mag_model2 = model2([I2, B2, K2], omega_plot)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 9))
    
    # Magnitude plot
    ax1.loglog(f_plot, mag_measured, 'b-', linewidth=2.5, label='Measured', alpha=0.8)
    ax1.loglog(f_plot, mag_model1, 'r--', linewidth=2, label=f'1st-order (err: {error1:.2f} dB)', alpha=0.8)
    if I2 is not None:
        ax1.loglog(f_plot, mag_model2, 'g-.', linewidth=2, label=f'2nd-order (err: {error2:.2f} dB)', alpha=0.8)
    
    ax1.grid(True, which='both', alpha=0.3)
    ax1.set_ylabel('Magnitude (rad/s)/Nm', fontsize=12, fontweight='bold')
    ax1.set_title('System Identification: Model Comparison', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11, loc='best')
    ax1.set_ylim([1e-3, 1e2])
    
    # Error plot
    mag_db_measured = 20 * np.log10(mag_measured + 1e-10)
    mag_db_model1 = 20 * np.log10(mag_model1 + 1e-10)
    error_plot1 = mag_db_model1 - mag_db_measured
    
    ax2.semilogx(f_plot, error_plot1, 'r--', linewidth=2, label='1st-order error', alpha=0.8)
    
    if I2 is not None:
        mag_db_model2 = 20 * np.log10(mag_model2 + 1e-10)
        error_plot2 = mag_db_model2 - mag_db_measured
        ax2.semilogx(f_plot, error_plot2, 'g-.', linewidth=2, label='2nd-order error', alpha=0.8)
    
    ax2.axhline(0, color='k', linestyle='-', alpha=0.3, linewidth=1)
    ax2.fill_between(f_plot, -2, 2, alpha=0.2, color='green', label='±2 dB band')
    ax2.grid(True, which='both', alpha=0.3)
    ax2.set_xlabel('Frequency (Hz)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Error (dB)', fontsize=12, fontweight='bold')
    ax2.set_ylim([-10, 10])
    ax2.legend(fontsize=11, loc='best')
    
    plt.tight_layout()
    output_file = os.path.splitext(csv_file)[0] + '_model_comparison.png'
    plt.savefig(output_file, dpi=150)
    print(f"✅ Comparison plot saved: {output_file}\n")
    plt.show()
    
    return {
        'model1': {'I': I1, 'B': B1, 'error': error1},
        'model2': {'I': I2, 'B': B2, 'K': K2, 'error': error2} if I2 else None,
        'better': better_model,
    }

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 compare_models.py <aligned_csv_file>")
        print("Example: python3 compare_models.py motor_log_aligned_20260608_170004.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    fit_models(csv_file)
