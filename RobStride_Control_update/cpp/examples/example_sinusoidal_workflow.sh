#!/bin/bash
# example_sinusoidal_workflow.sh - Example workflow for sinusoidal testing

# This script demonstrates a complete testing workflow:
# 1. Run sinusoidal tests with different parameters
# 2. Generate plots for analysis
# 3. Create comparison reports

set -e

MOTOR_ID=${1:-11}
OUTPUT_DIR="sinusoidal_test_results_$(date +%Y%m%d_%H%M%S)"

echo "🎯 SINUSOIDAL ACTUATOR TEST WORKFLOW"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Motor ID:    $MOTOR_ID"
echo "Output Dir:  $OUTPUT_DIR"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Check if binary exists
if [ ! -f "./sinusoidal_actuator_test" ]; then
    echo "❌ Binary not found. Building..."
    bash build_sinusoidal_test.sh
fi

if [ ! -f "./plot_sinusoidal_test.py" ]; then
    echo "❌ Plot script not found"
    exit 1
fi

# Make plot script executable
chmod +x plot_sinusoidal_test.py

echo ""
echo "📋 TEST 1: Simple Sine Wave (Baseline)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Parameters: freq=1.0Hz, amp=30°, kp=10, kd=2"
echo ""
sudo ./sinusoidal_actuator_test $MOTOR_ID simple_sin 10 2 1.0 30
LOGFILE_1=$(ls -t sinusoidal_test_${MOTOR_ID}_simple_sin*.csv | head -1)
cp "$LOGFILE_1" "$OUTPUT_DIR/"
echo "✅ Log saved: $LOGFILE_1"
echo ""

# Small delay between tests
echo "⏳ Waiting 5 seconds before next test..."
sleep 5

echo "📋 TEST 2: Progressive Frequency (Sweep)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Parameters: sin_time_square with increasing frequency"
echo ""
sudo ./sinusoidal_actuator_test $MOTOR_ID sin_time_square 10 2 2.0 45
LOGFILE_2=$(ls -t sinusoidal_test_${MOTOR_ID}_sin_time_square*.csv | head -1)
cp "$LOGFILE_2" "$OUTPUT_DIR/"
echo "✅ Log saved: $LOGFILE_2"
echo ""

# Small delay between tests
echo "⏳ Waiting 5 seconds before next test..."
sleep 5

echo "📋 TEST 3: Multi-Frequency (Harmonics)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Parameters: sin_sin with fundamental + 2nd harmonic"
echo ""
sudo ./sinusoidal_actuator_test $MOTOR_ID sin_sin 8 1.5 1.5 30
LOGFILE_3=$(ls -t sinusoidal_test_${MOTOR_ID}_sin_sin*.csv | head -1)
cp "$LOGFILE_3" "$OUTPUT_DIR/"
echo "✅ Log saved: $LOGFILE_3"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ ALL TESTS COMPLETED"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "📊 GENERATING PLOTS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

PLOTS_DIR="${OUTPUT_DIR}/plots"
mkdir -p "$PLOTS_DIR"

echo "  [1/3] Analyzing simple_sin test..."
python3 plot_sinusoidal_test.py "$LOGFILE_1" --save --output "$PLOTS_DIR" > /dev/null 2>&1

echo "  [2/3] Analyzing sin_time_square test..."
python3 plot_sinusoidal_test.py "$LOGFILE_2" --save --output "$PLOTS_DIR" > /dev/null 2>&1

echo "  [3/3] Analyzing sin_sin test..."
python3 plot_sinusoidal_test.py "$LOGFILE_3" --save --output "$PLOTS_DIR" > /dev/null 2>&1

echo "✅ Plots generated"
echo ""

# Create summary report
REPORT="$OUTPUT_DIR/SUMMARY.txt"
cat > "$REPORT" << EOF
═══════════════════════════════════════════════════════════
  SINUSOIDAL ACTUATOR TEST SUMMARY
═══════════════════════════════════════════════════════════

Test Date:    $(date)
Motor ID:     $MOTOR_ID
Test Count:   3

───────────────────────────────────────────────────────────
TEST 1: SIMPLE SINE WAVE
───────────────────────────────────────────────────────────
File:         $(basename "$LOGFILE_1")
Trajectory:   Single frequency sinusoid
Parameters:   Frequency=1.0 Hz, Amplitude=30°, KP=10, KD=2
Purpose:      Baseline performance characterization

EOF

python3 plot_sinusoidal_test.py "$LOGFILE_1" >> "$REPORT" 2>&1 || true

cat >> "$REPORT" << EOF

───────────────────────────────────────────────────────────
TEST 2: PROGRESSIVE FREQUENCY SWEEP
───────────────────────────────────────────────────────────
File:         $(basename "$LOGFILE_2")
Trajectory:   sin(t²) - frequency increases over time
Parameters:   Base Frequency=2.0 Hz, Amplitude=45°, KP=10, KD=2
Purpose:      Characterize response across frequency range

EOF

python3 plot_sinusoidal_test.py "$LOGFILE_2" >> "$REPORT" 2>&1 || true

cat >> "$REPORT" << EOF

───────────────────────────────────────────────────────────
TEST 3: MULTI-FREQUENCY HARMONICS
───────────────────────────────────────────────────────────
File:         $(basename "$LOGFILE_3")
Trajectory:   sin(ωt) + 0.5·sin(2ωt) - dual frequencies
Parameters:   Frequency=1.5 Hz, Amplitude=30°, KP=8, KD=1.5
Purpose:      Test response to harmonic content

EOF

python3 plot_sinusoidal_test.py "$LOGFILE_3" >> "$REPORT" 2>&1 || true

cat >> "$REPORT" << EOF

───────────────────────────────────────────────────────────
OUTPUT FILES
───────────────────────────────────────────────────────────

CSV Data:
  • $(basename "$LOGFILE_1")
  • $(basename "$LOGFILE_2")
  • $(basename "$LOGFILE_3")

Plots (PNG):
  • simple_sin_position.png
  • simple_sin_error.png
  • simple_sin_analysis.png
  • sin_time_square_position.png
  • sin_time_square_error.png
  • sin_time_square_analysis.png
  • sin_sin_position.png
  • sin_sin_error.png
  • sin_sin_analysis.png

───────────────────────────────────────────────────────────
NEXT STEPS
───────────────────────────────────────────────────────────

1. Review plots in the 'plots' subdirectory

2. Compare tracking performance across tests:
   • Simple sine: Baseline with fixed frequency
   • Progressive sweep: How response changes over frequency
   • Multi-harmonic: Handling of complex inputs

3. Examine error characteristics:
   • Peak error magnitude
   • Error distribution (lag vs lead)
   • Resonance peaks

4. Consider parameter adjustment:
   • If oscillatory: Reduce KP or increase KD
   • If sluggish: Increase KP or reduce KD
   • If unstable: Reduce both gains

═══════════════════════════════════════════════════════════
EOF

echo "📄 Summary report: $REPORT"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📁 RESULTS LOCATION: $OUTPUT_DIR"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📂 Directory structure:"
echo "   $OUTPUT_DIR/"
echo "   ├── SUMMARY.txt              (This report)"
echo "   ├── sinusoidal_test_*.csv    (Raw data)"
echo "   └── plots/"
echo "       ├── *_position.png       (Position tracking)"
echo "       ├── *_error.png          (Error analysis)"
echo "       └── *_analysis.png       (Phase & power)"
echo ""
echo "💡 View plots:"
echo "   ls -la $OUTPUT_DIR/plots/"
echo ""
echo "✨ Interactive viewing (optional):"
echo "   python3 plot_sinusoidal_test.py $LOGFILE_1 --show"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
