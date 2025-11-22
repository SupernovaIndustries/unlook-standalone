#!/bin/bash
#
# Unlook 3D Scanner - Installation Script
# Installs dependencies, builds the project, and creates the global 'unlook' command
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_PREFIX="/usr/local"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[*]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[+]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[-]${NC} $1"
}

# Check if running as root for installation step
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        print_warning "Some operations require sudo. You may be prompted for your password."
    fi
}

# Install system dependencies
install_dependencies() {
    print_status "Installing system dependencies..."

    sudo apt update
    sudo apt install -y \
        build-essential cmake ninja-build pkg-config \
        qtbase5-dev qttools5-dev qttools5-dev-tools libqt5svg5-dev \
        libopencv-dev libopencv-contrib-dev \
        libcamera-dev libcamera-tools \
        libyaml-cpp-dev libfmt-dev \
        libgpiod-dev libi2c-dev i2c-tools \
        git

    print_success "Dependencies installed"
}

# Configure libcamera timeout for Raspberry Pi 5
configure_libcamera_timeout() {
    print_status "Configuring libcamera timeout..."

    TIMEOUT_FILE="/home/${SUDO_USER:-$USER}/timeout.yaml"

    if [ -f "/usr/share/libcamera/pipeline/rpi/pisp/example.yaml" ]; then
        # Pi5/CM5 with PiSP
        cp /usr/share/libcamera/pipeline/rpi/pisp/example.yaml "$TIMEOUT_FILE"
    elif [ -f "/usr/share/libcamera/pipeline/rpi/vc4/example.yaml" ]; then
        # Pi4/CM4 with VC4
        cp /usr/share/libcamera/pipeline/rpi/vc4/example.yaml "$TIMEOUT_FILE"
    else
        print_warning "Could not find libcamera example.yaml - skipping timeout configuration"
        return
    fi

    # Set timeout to 60 seconds
    sed -i 's/# "camera_timeout_value_ms": 0/"camera_timeout_value_ms": 60000/' "$TIMEOUT_FILE"

    # Add to bashrc if not already present
    BASHRC="/home/${SUDO_USER:-$USER}/.bashrc"
    if ! grep -q "LIBCAMERA_RPI_CONFIG_FILE" "$BASHRC" 2>/dev/null; then
        echo "export LIBCAMERA_RPI_CONFIG_FILE=$TIMEOUT_FILE" >> "$BASHRC"
    fi

    print_success "Libcamera timeout configured: $TIMEOUT_FILE"
}

# Build the project
build_project() {
    print_status "Building Unlook..."

    cd "$SCRIPT_DIR"

    # Clean build directory if requested
    if [ "$1" == "--clean" ]; then
        print_status "Cleaning build directory..."
        rm -rf "$BUILD_DIR"
    fi

    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    # Configure with CMake
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DBUILD_EXAMPLES=ON \
        -DBUILD_GUI=ON

    # Build with parallel jobs
    JOBS=$(nproc)
    make -j"$JOBS"

    print_success "Build completed"
}

# Install the global 'unlook' command
install_global_command() {
    print_status "Installing global 'unlook' command..."

    # Create the launcher script
    LAUNCHER_SCRIPT="/usr/local/bin/unlook"

    sudo tee "$LAUNCHER_SCRIPT" > /dev/null << 'EOF'
#!/bin/bash
#
# Unlook 3D Scanner Launcher
#

# Set library paths
export LD_LIBRARY_PATH="/home/alessandro/unlook-standalone/build/src:$LD_LIBRARY_PATH"

# Set libcamera config if exists
if [ -f "/home/alessandro/timeout.yaml" ]; then
    export LIBCAMERA_RPI_CONFIG_FILE="/home/alessandro/timeout.yaml"
fi

# Launch the application
exec /home/alessandro/unlook-standalone/build/src/gui/unlook_scanner "$@"
EOF

    sudo chmod +x "$LAUNCHER_SCRIPT"

    print_success "Global command 'unlook' installed at $LAUNCHER_SCRIPT"
}

# Create desktop entry (optional)
create_desktop_entry() {
    print_status "Creating desktop entry..."

    DESKTOP_FILE="/usr/share/applications/unlook.desktop"

    sudo tee "$DESKTOP_FILE" > /dev/null << EOF
[Desktop Entry]
Name=Unlook 3D Scanner
Comment=Professional 3D Scanner Application
Exec=/usr/local/bin/unlook
Icon=camera-photo
Terminal=false
Type=Application
Categories=Graphics;3DGraphics;Engineering;
Keywords=3D;scanner;stereo;camera;
EOF

    print_success "Desktop entry created"
}

# Main installation
main() {
    echo ""
    echo "=========================================="
    echo "   Unlook 3D Scanner - Installer"
    echo "=========================================="
    echo ""

    check_sudo

    # Parse arguments
    SKIP_DEPS=false
    CLEAN_BUILD=false

    for arg in "$@"; do
        case $arg in
            --skip-deps)
                SKIP_DEPS=true
                ;;
            --clean)
                CLEAN_BUILD=true
                ;;
            --help)
                echo "Usage: $0 [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  --skip-deps    Skip installing system dependencies"
                echo "  --clean        Clean build directory before building"
                echo "  --help         Show this help message"
                exit 0
                ;;
        esac
    done

    # Step 1: Install dependencies
    if [ "$SKIP_DEPS" = false ]; then
        install_dependencies
    else
        print_warning "Skipping dependency installation"
    fi

    # Step 2: Configure libcamera timeout
    configure_libcamera_timeout

    # Step 3: Build the project
    if [ "$CLEAN_BUILD" = true ]; then
        build_project --clean
    else
        build_project
    fi

    # Step 4: Install global command
    install_global_command

    # Step 5: Create desktop entry
    create_desktop_entry

    echo ""
    echo "=========================================="
    print_success "Installation completed!"
    echo "=========================================="
    echo ""
    echo "You can now run Unlook from anywhere with:"
    echo ""
    echo "    unlook"
    echo ""
    echo "Or find it in your application menu."
    echo ""
    echo "Note: You may need to restart your terminal or run:"
    echo "    source ~/.bashrc"
    echo ""
}

main "$@"
