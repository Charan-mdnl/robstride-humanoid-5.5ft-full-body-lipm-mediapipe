#!/bin/bash
# build_sinusoidal_test.sh - Build the sinusoidal actuator test

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "🔨 Building Sinusoidal Actuator Test..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check for required headers
if [ ! -f "../include/motor_control.hpp" ]; then
    echo "❌ Error: ../include/motor_control.hpp not found"
    exit 1
fi

# Compile
echo "📦 Compiling C++ code..."
g++ -std=c++17 -Wall -Wextra -O2 \
    -I../include \
    -o sinusoidal_actuator_test \
    sinusoidal_actuator_test.cpp \
    ../src/can_interfaces.cpp \
    -pthread

echo "✅ Build successful!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📝 Usage:"
echo "   sudo ./sinusoidal_actuator_test <motor_id> <trajectory> [kp] [kd] [freq] [amp]"
echo ""
echo "📚 Examples:"
echo "   sudo ./sinusoidal_actuator_test 11 sin_time_square 10 2 2.0 45"
echo "   sudo ./sinusoidal_actuator_test 11 sin_sin 8 1.5 1.5 30"
echo "   sudo ./sinusoidal_actuator_test 11 simple_sin 10 2 1.0 30"
echo ""
echo "📊 Plot results with:"
echo "   python3 plot_sinusoidal_test.py <logfile.csv> --show"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
