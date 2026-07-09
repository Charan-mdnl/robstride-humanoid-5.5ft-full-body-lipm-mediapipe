#!/bin/bash
# tianrking Control Environment Setup Script
# This script sets up the development environment for tianrking motor control

set -e

echo "🚀 Setting up tianrking Control development environment..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[i]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    print_warning "Running as root. Consider running as regular user with sudo."
fi

# Update system packages
print_info "Updating system packages..."
sudo apt-get update

# Install CAN utilities
print_info "Installing CAN utilities..."
sudo apt-get install -y can-utils

# Install Python development environment
print_info "Setting up Python environment..."
sudo apt-get install -y python3 python3-pip python3-venv

# Install C++ development environment
print_info "Setting up C++ environment..."
sudo apt-get install -y build-essential cmake pkg-config

# Install Rust
if ! command -v rustc &> /dev/null; then
    print_info "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
else
    print_status "Rust already installed"
fi

# Create Python virtual environment
if [ ! -d "venv" ]; then
    print_info "Creating Python virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment and install Python dependencies
print_info "Installing Python dependencies..."
source venv/bin/activate
pip install -r python/requirements.txt

# Build C++ components
print_info "Building C++ components..."
cd cpp
mkdir -p build
cd build
cmake ..
make
cd ../..

# Build Rust components
print_info "Building Rust components..."
cd rust
cargo build --release
cd ..

# Setup CAN interface
print_info "Setting up CAN interface..."
if ip link show can0 &> /dev/null; then
    print_status "CAN interface can0 already exists"
else
    print_warning "CAN interface can0 not found. You may need to configure your hardware."
fi

# Set permissions
print_info "Setting permissions..."
sudo usermod -a -G dialout $USER 2>/dev/null || true

# Create symbolic links for easy access
print_info "Creating symbolic links..."
sudo ln -sf $(pwd)/cpp/build/robstride-mit-position /usr/local/bin/robstride-cpp 2>/dev/null || true
sudo ln -sf $(pwd)/rust/target/release/robstride-mit-position /usr/local/bin/robstride-rust 2>/dev/null || true

print_status "Setup completed successfully!"
echo
print_info "Next steps:"
echo "1. Reboot your system or log out and log back in to apply group changes"
echo "2. Connect your CAN hardware"
echo "3. Run: sudo ip link set can0 type can bitrate 1000000"
echo "4. Run: sudo ip link set up can0"
echo "5. Test with: python python/src/position_control.py 11"
echo
print_warning "Remember to use sudo when accessing CAN devices!"