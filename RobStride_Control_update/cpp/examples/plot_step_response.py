#!/usr/bin/env python3
"""
Plot step response data from aligned CSV (position control tests)
Shows: target angle, actual angle, velocity, torque
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def plot_step_response(csv_file):
    """Load aligned CSV and plot position control step response"""
    
    # Load data
    df = pd.read_csv(csv_file)
    print(f"Loaded: {csv_file}")
    print(f"Shape: {df.shape}")
    print(f"Columns: {list(df.columns)}")
    
    # Find motor columns (assuming single motor, look for target_deg, actual_deg, vel_rad, torque_Nm)
    target_col = None
    actual_col = None
    vel_col = None
    torque_col = None
    
    for col in df.columns:
        if 'target_deg' in col and target_col is None:
            target_col = col
        if 'actual_deg' in col and actual_col is None:
            actual_col = col
        if 'vel_rad' in col or 'vel_dps' in col and vel_col is None:
            vel_col = col
        if 'torque_nm' in col.lower() or 'torque_nm' in col and torque_col is None:
            torque_col = col
    
    print(f"\nDetected columns:")
    print(f"  Target: {target_col}")
    print(f"  Actual: {actual_col}")
    print(f"  Velocity: {vel_col}")
    print(f"  Torque: {torque_col}")
    
    # Time axis (convert from ms to seconds if needed)
    if 'timestamp_ms' in df.columns:
        t = df['timestamp_ms'].values / 1000.0
    elif 'timestamp' in df.columns:
        t = df['timestamp'].values
    else:
        t = df.index.values / 1000.0  # Assume 1000 Hz if no timestamp
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Plot 1: Position (target vs actual)
    if target_col and actual_col:
        axes[0].plot(t, df[target_col], 'r--', label='Target', linewidth=2, alpha=0.7)
        axes[0].plot(t, df[actual_col], 'b-', label='Actual', linewidth=1.5)
        axes[0].set_ylabel('Angle (degrees)', fontsize=11)
        axes[0].set_title('Position Control Step Response', fontsize=12, fontweight='bold')
        axes[0].legend(loc='best')
        axes[0].grid(True, alpha=0.3)
        
        # Calculate tracking error
        error = df[actual_col] - df[target_col]
        axes[0].text(0.02, 0.95, f'Max Error: {error.abs().max():.2f}°\nRMS Error: {(error**2).mean()**0.5:.2f}°',
                    transform=axes[0].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5), fontsize=9)
    
    # Plot 2: Velocity
    if vel_col:
        axes[1].plot(t, df[vel_col], 'g-', linewidth=1.5)
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=11)
        axes[1].set_title('Angular Velocity Response', fontsize=12, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Torque
    if torque_col:
        axes[2].plot(t, df[torque_col], 'orange', linewidth=1.5)
        axes[2].set_ylabel('Torque (Nm)', fontsize=11)
        axes[2].set_xlabel('Time (s)', fontsize=11)
        axes[2].set_title('Command Torque Response', fontsize=12, fontweight='bold')
        axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save figure
    output_file = csv_file.replace('.csv', '_step_response.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Saved: {output_file}")
    
    # Also show
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # Use most recent aligned CSV in current directory
        import glob
        files = glob.glob('motor_log_aligned_*.csv')
        if files:
            csv_file = sorted(files)[-1]
            print(f"Using most recent: {csv_file}")
        else:
            print("Usage: python3 plot_step_response.py <csv_file>")
            sys.exit(1)
    
    plot_step_response(csv_file)
