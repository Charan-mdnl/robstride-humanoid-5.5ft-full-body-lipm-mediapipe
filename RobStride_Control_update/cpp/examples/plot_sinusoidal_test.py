#!/usr/bin/env python3
"""
Sinusoidal Actuator Test Plotter
Visualizes results from sinusoidal_actuator_test.cpp

Usage:
    python3 plot_sinusoidal_test.py <logfile.csv> [--show] [--save]

Example:
    python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square_20250520_120000.csv --show
"""

import pandas as pd
import numpy as np
import sys
import argparse
from pathlib import Path


def load_and_validate_data(filepath):
    """Load CSV data and validate structure"""
    try:
        df = pd.read_csv(filepath)
        required_cols = ['time_s', 'target_pos_deg', 'actual_pos_deg', 
                        'target_vel_deg_s', 'actual_vel_deg_s', 'actual_torque_nm']
        missing = [c for c in required_cols if c not in df.columns]
        if missing:
            raise ValueError(f"Missing columns: {missing}")
        return df
    except Exception as e:
        print(f"❌ Error loading {filepath}: {e}")
        sys.exit(1)


def calculate_metrics(df):
    """Calculate key performance metrics"""
    error = np.abs(df['target_pos_deg'] - df['actual_pos_deg'])
    
    metrics = {
        'mae': error.mean(),
        'rmse': np.sqrt((error ** 2).mean()),
        'max_error': error.max(),
        'settling_time': None,  # Find when error < 5% final value
        'min_torque': df['actual_torque_nm'].min(),
        'max_torque': df['actual_torque_nm'].max(),
        'rms_velocity': np.sqrt((df['actual_vel_deg_s'] ** 2).mean()),
        'samples': len(df),
    }
    
    return metrics


def create_figures(df, logfile):
    """Create comprehensive visualization figures"""
    import matplotlib.pyplot as plt

    # Figure 1: Position Tracking
    fig1, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig1.suptitle(f'Sinusoidal Actuator Test - {Path(logfile).stem}', fontsize=14, fontweight='bold')
    
    # Top: Position
    ax = axes[0]
    ax.plot(df['time_s'], df['target_pos_deg'], 'g--', linewidth=2, label='Target', alpha=0.8)
    ax.plot(df['time_s'], df['actual_pos_deg'], 'b-', linewidth=1.5, label='Actual', alpha=0.9)
    ax.fill_between(df['time_s'], 
                     df['target_pos_deg'], 
                     df['actual_pos_deg'],
                     alpha=0.2, color='red', label='Error')
    ax.set_ylabel('Position (deg)', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.set_title('Position Tracking', fontsize=12)
    
    # Bottom: Velocity
    ax = axes[1]
    ax.plot(df['time_s'], df['target_vel_deg_s'], 'g--', linewidth=2, label='Target Velocity', alpha=0.8)
    ax.plot(df['time_s'], df['actual_vel_deg_s'], 'c-', linewidth=1.5, label='Actual Velocity', alpha=0.9)
    ax.set_ylabel('Velocity (deg/s)', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.set_title('Velocity Tracking', fontsize=12)
    
    plt.tight_layout()
    
    # Figure 2: Error Analysis
    fig2, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig2.suptitle(f'Error & Torque Analysis - {Path(logfile).stem}', fontsize=14, fontweight='bold')
    
    # Top: Tracking Error
    error = df['target_pos_deg'] - df['actual_pos_deg']
    ax = axes[0]
    ax.plot(df['time_s'], error, 'r-', linewidth=1.5, alpha=0.8)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.fill_between(df['time_s'], error, alpha=0.2, color='red')
    ax.set_ylabel('Error (deg)', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_title('Tracking Error (Target - Actual)', fontsize=12)
    
    # Bottom: Torque
    ax = axes[1]
    ax.plot(df['time_s'], df['actual_torque_nm'], 'purple', linewidth=1.5, alpha=0.8)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_ylabel('Torque (Nm)', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_title('Motor Torque Output', fontsize=12)
    
    plt.tight_layout()
    
    # Figure 3: Phase Plot & Power
    fig3, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig3.suptitle(f'Phase & Power Analysis - {Path(logfile).stem}', fontsize=14, fontweight='bold')
    
    # Left: Phase plot (Velocity vs Position)
    ax = axes[0]
    scatter = ax.scatter(df['actual_pos_deg'], df['actual_vel_deg_s'], 
                        c=df['time_s'], cmap='viridis', s=20, alpha=0.6)
    ax.set_xlabel('Position (deg)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Velocity (deg/s)', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_title('Phase Plot (Velocity vs Position)', fontsize=12)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Time (s)', fontsize=10)
    
    # Right: Power
    ax = axes[1]
    power = df['actual_torque_nm'] * df['actual_vel_deg_s']
    ax.plot(df['time_s'], power, 'orange', linewidth=1.5, alpha=0.8)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.fill_between(df['time_s'], power, alpha=0.2, color='orange')
    ax.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Power (W)', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_title('Instantaneous Power (τ × ω)', fontsize=12)
    
    plt.tight_layout()
    
    return fig1, fig2, fig3


def print_metrics(df, logfile):
    """Print performance metrics"""
    metrics = calculate_metrics(df)
    
    print("\n" + "="*60)
    print(f"📊 SINUSOIDAL TEST RESULTS: {Path(logfile).name}")
    print("="*60)
    print(f"Duration:           {df['time_s'].max():.2f} seconds")
    print(f"Samples:            {metrics['samples']}")
    print(f"Sample Rate:        {metrics['samples']/df['time_s'].max():.1f} Hz")
    print()
    print("TRACKING PERFORMANCE:")
    print(f"  Mean Absolute Error (MAE):  {metrics['mae']:.4f}°")
    print(f"  Root Mean Squared Error:    {metrics['rmse']:.4f}°")
    print(f"  Maximum Error:              {metrics['max_error']:.4f}°")
    print()
    print("TORQUE CHARACTERISTICS:")
    print(f"  Min Torque:                 {metrics['min_torque']:.4f} Nm")
    print(f"  Max Torque:                 {metrics['max_torque']:.4f} Nm")
    print(f"  Torque Range:               {metrics['max_torque'] - metrics['min_torque']:.4f} Nm")
    print()
    print("VELOCITY CHARACTERISTICS:")
    print(f"  RMS Velocity:               {metrics['rms_velocity']:.4f} deg/s")
    print(f"  Max Velocity:               {df['actual_vel_deg_s'].max():.4f} deg/s")
    print(f"  Min Velocity:               {df['actual_vel_deg_s'].min():.4f} deg/s")
    print()
    print("CONTROL PARAMETERS:")
    if 'kp' in df.columns:
        print(f"  KP (Proportional):          {df['kp'].iloc[0]:.2f}")
    if 'kd' in df.columns:
        print(f"  KD (Derivative):            {df['kd'].iloc[0]:.2f}")
    print("="*60 + "\n")


def save_figures(fig1, fig2, fig3, logfile, output_dir='./sinusoidal_plots'):
    """Save figures to disk"""
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)
    
    base_name = Path(logfile).stem
    
    fig1_path = output_path / f"{base_name}_position.png"
    fig2_path = output_path / f"{base_name}_error.png"
    fig3_path = output_path / f"{base_name}_analysis.png"
    
    fig1.savefig(fig1_path, dpi=150, bbox_inches='tight')
    print(f"✅ Saved: {fig1_path}")
    
    fig2.savefig(fig2_path, dpi=150, bbox_inches='tight')
    print(f"✅ Saved: {fig2_path}")
    
    fig3.savefig(fig3_path, dpi=150, bbox_inches='tight')
    print(f"✅ Saved: {fig3_path}")
    
    return fig1_path.parent


def try_interactive_backend():
    """
    Attempt to switch to an interactive matplotlib backend.
    Returns True if a working interactive backend was found, False otherwise.
    """
    import matplotlib
    import matplotlib.pyplot as plt

    interactive_backends = ['TkAgg', 'Qt5Agg', 'Qt6Agg', 'wxAgg', 'GTK3Agg']

    for backend in interactive_backends:
        try:
            matplotlib.use(backend)
            # Force a test draw to confirm it actually works
            fig = plt.figure()
            plt.close(fig)
            return True
        except Exception:
            continue

    return False


def main():
    parser = argparse.ArgumentParser(
        description='Plot sinusoidal actuator test results',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square_20250520_120000.csv --show
  python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_sin_20250520_120000.csv --save
  python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_sin_20250520_120000.csv --show --save
        """
    )
    
    parser.add_argument('logfile', help='CSV log file from sinusoidal_actuator_test')
    parser.add_argument('--show', action='store_true', help='Display plots interactively')
    parser.add_argument('--save', action='store_true', help='Save plots to PNG files')
    parser.add_argument('--output', default='./sinusoidal_plots', help='Output directory for plots')
    
    args = parser.parse_args()

    # Resolve show/save intent:
    # If neither flag is given, default to --save (safe for headless environments).
    want_show = args.show
    want_save = args.save or (not args.show)  # save is the safe default

    # If the user wants interactive display, try to set an interactive backend
    # BEFORE importing pyplot for the first time.
    if want_show:
        import matplotlib
        current = matplotlib.get_backend().lower()
        if current == 'agg':
            print("⚙️  Attempting to switch to an interactive backend...")
            if try_interactive_backend():
                print(f"✅ Using backend: {matplotlib.get_backend()}")
            else:
                print("⚠️  No interactive display available (headless environment).")
                print("    Falling back to --save instead of --show.")
                want_show = False
                want_save = True

    import matplotlib.pyplot as plt  # import after backend is settled

    # Load and validate
    print(f"📂 Loading: {args.logfile}")
    df = load_and_validate_data(args.logfile)
    
    # Print metrics
    print_metrics(df, args.logfile)
    
    # Create figures
    print("🎨 Creating visualizations...")
    fig1, fig2, fig3 = create_figures(df, args.logfile)
    
    # Save
    if want_save:
        output_dir = save_figures(fig1, fig2, fig3, args.logfile, args.output)
        print(f"\n💾 Plots saved to: {output_dir}")

    # Show
    if want_show:
        plt.show()
    else:
        plt.close('all')


if __name__ == '__main__':
    main()
