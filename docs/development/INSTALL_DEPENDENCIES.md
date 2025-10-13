# UNLOOK 3D SCANNER - DEPENDENCY INSTALLATION GUIDE

**CRITICAL**: Install ALL these dependencies BEFORE building the Unlook 3D Scanner.

This guide provides exact installation commands for Raspberry Pi OS (Debian/Ubuntu based systems).

---

## 🚨 MANDATORY SYSTEM DEPENDENCIES

### **1. Build Tools and Compilers**
```bash
sudo apt update
sudo apt install -y build-essential
sudo apt install -y cmake
sudo apt install -y pkg-config
sudo apt install -y git
sudo apt install -y wget
sudo apt install -y curl
```

### **2. Threading and System Libraries**
```bash
# Install threading libraries
sudo apt install -y libc6-dev
sudo apt install -y libpthread-stubs0-dev

# Install standard C++ libraries
sudo apt install -y libstdc++-12-dev
sudo apt install -y libc++-dev
```

### **3. OpenCV Development Libraries (REQUIRED)**
```bash
# Install OpenCV development packages
sudo apt install -y libopencv-dev
sudo apt install -y libopencv-contrib-dev

# Core OpenCV libraries
sudo apt install -y libopencv-core-dev
sudo apt install -y libopencv-imgproc-dev
sudo apt install -y libopencv-imgcodecs-dev
sudo apt install -y libopencv-calib3d-dev
sudo apt install -y libopencv-features2d-dev

# Additional OpenCV components
sudo apt install -y libopencv-highgui-dev
sudo apt install -y libopencv-videoio-dev
```

### **4. Qt5 Development Libraries (GUI)**
```bash
# Qt5 base development
sudo apt install -y qtbase5-dev
sudo apt install -y qtbase5-dev-tools
sudo apt install -y qt5-qmake
sudo apt install -y qttools5-dev
sudo apt install -y qttools5-dev-tools

# Qt5 additional components
sudo apt install -y libqt5widgets5
sudo apt install -y libqt5core5a
sudo apt install -y libqt5gui5
sudo apt install -y libqt5opengl5-dev
```

### **5. libcamera Development (Camera Control)**
```bash
# libcamera development (standard version)
sudo apt install -y libcamera-dev
sudo apt install -y libcamera-tools
sudo apt install -y libcamera-apps

# Note: libcamera-sync custom version is already installed at /usr/local/
# The build system will automatically detect and prefer the custom version
```

### **6. Additional Image Processing Libraries**
```bash
# JPEG, PNG, TIFF support
sudo apt install -y libjpeg-dev
sudo apt install -y libpng-dev
sudo apt install -y libtiff-dev

# Video codec libraries
sudo apt install -y libavcodec-dev
sudo apt install -y libavformat-dev
sudo apt install -y libswscale-dev
```

### **7. Mathematical Libraries**
```bash
# BLAS/LAPACK for mathematical operations
sudo apt install -y libatlas-base-dev
sudo apt install -y liblapack-dev
sudo apt install -y libeigen3-dev

# OpenMP for parallel processing
sudo apt install -y libomp-dev
```

### **8. Development Tools and Documentation**
```bash
# Documentation generation
sudo apt install -y doxygen
sudo apt install -y graphviz

# Additional development tools
sudo apt install -y gdb
sudo apt install -y valgrind
```

---

## 🎯 COMPLETE ONE-COMMAND INSTALLATION

Copy and paste this single command to install ALL dependencies:

```bash
sudo apt update && sudo apt install -y \
    build-essential cmake pkg-config git wget curl \
    libc6-dev libpthread-stubs0-dev libstdc++-12-dev libc++-dev \
    libopencv-dev libopencv-contrib-dev \
    libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev \
    libopencv-calib3d-dev libopencv-features2d-dev \
    libopencv-highgui-dev libopencv-videoio-dev \
    qtbase5-dev qtbase5-dev-tools qt5-qmake qttools5-dev qttools5-dev-tools \
    libqt5widgets5 libqt5core5a libqt5gui5 libqt5opengl5-dev \
    libcamera-dev libcamera-tools libcamera-apps \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libatlas-base-dev liblapack-dev libeigen3-dev libomp-dev \
    doxygen graphviz gdb valgrind
```

---

## 🔍 DEPENDENCY VERIFICATION

After installation, verify all dependencies are correctly installed:

### **1. Verify Build Tools**
```bash
# Check compilers
gcc --version          # Should show GCC 12.x
g++ --version          # Should show G++ 12.x
cmake --version        # Should show CMake 3.25+

# Check build tools
make --version
pkg-config --version
```

### **2. Verify OpenCV Installation**
```bash
# Check OpenCV version
pkg-config --modversion opencv4
# Should show: 4.6.0 or similar

# Check OpenCV libraries
pkg-config --libs opencv4
# Should show: -lopencv_core -lopencv_imgproc ... (many libraries)

# Find OpenCV CMake config
find /usr -name "OpenCVConfig.cmake" 2>/dev/null
# Should show: /usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
```

### **3. Verify Qt5 Installation**
```bash
# Check Qt5 version
qmake -v
# Should show: Using Qt version 5.15.x

# Check Qt5 libraries
pkg-config --modversion Qt5Core Qt5Widgets Qt5OpenGL
# Should show version numbers for each
```

### **4. Verify libcamera**
```bash
# Check system libcamera
pkg-config --modversion libcamera
# Should show version

# Check custom libcamera-sync (preferred)
ls -la /usr/local/lib/libcamera*
ls -la /usr/local/include/libcamera/
# Should show libcamera-sync files if installed
```

### **5. Test Complete Dependency Chain**
```bash
# Create a simple test
cd /home/alessandro/unlook-standalone
mkdir -p test_deps && cd test_deps

# Create test CMakeLists.txt
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(DependencyTest)
set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc calib3d)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA libcamera)

message(STATUS "All dependencies found successfully!")
EOF

# Test configuration
cmake .
# Should complete without errors

# Cleanup
cd .. && rm -rf test_deps
```

---

## 🚨 TROUBLESHOOTING

### **Common Issues and Solutions:**

#### **1. Threading Issues**
```bash
# If pthread errors occur:
sudo apt install -y libc6-dev libpthread-stubs0-dev
sudo ldconfig
```

#### **2. OpenCV Not Found**
```bash
# If OpenCV CMake files missing:
sudo apt install -y libopencv-dev libopencv-contrib-dev
sudo apt install --reinstall libopencv-dev

# Check installation:
dpkg -L libopencv-dev | grep cmake
```

#### **3. Qt5 Issues**
```bash
# If Qt5 not found:
sudo apt install -y qtbase5-dev qttools5-dev
export Qt5_DIR="/usr/lib/aarch64-linux-gnu/cmake/Qt5"
```

#### **4. libcamera Conflicts**
```bash
# If libcamera version conflicts:
# The build system will automatically prefer /usr/local/ (libcamera-sync)
# over system libcamera. No action needed.
```

#### **5. CMake Version Issues**
```bash
# If CMake too old:
sudo apt install -y cmake
cmake --version  # Should be 3.16+

# If still old, install from official source:
wget https://github.com/Kitware/CMake/releases/download/v3.26.0/cmake-3.26.0-linux-aarch64.tar.gz
sudo tar -xzf cmake-3.26.0-linux-aarch64.tar.gz -C /opt/
sudo ln -sf /opt/cmake-3.26.0-linux-aarch64/bin/cmake /usr/local/bin/cmake
```

---

## ✅ BUILD VERIFICATION

After installing all dependencies, verify the build works:

```bash
cd /home/alessandro/unlook-standalone

# Clean any previous build attempts
rm -rf build/

# Test build configuration
./build.sh --clean -j 4

# Expected output should show:
# - "Build type: Release"
# - "Found system libcamera: X.X.X" or "Using system libcamera-sync"
# - "OpenCV version: 4.6.0"
# - "Qt5 version: 5.15.x" (if GUI enabled)
# - Successful compilation and linking
```

### **Expected Build Output:**
```
=== Unlook 3D Scanner Build ===
Build type: Release
Parallel jobs: 4

Found system libcamera: 0.5.1
Configuring with CMake...
-- OpenCV version: 4.6.0
-- Qt5 version: 5.15.8
-- libcamera: 0.5.1
-- Building...
[100%] Built target unlook
[100%] Built target unlook_scanner
```

### **Expected Build Products:**
```
build/
├── bin/unlook_scanner          # Main GUI application
├── lib/libunlook.so           # Shared API library
├── include/unlook/            # API headers
└── examples/                  # Example applications
```

---

## 🎯 FINAL VALIDATION

Run these commands to ensure everything is ready:

```bash
# 1. Verify build completed successfully
ls -la build/bin/unlook_scanner
ls -la build/lib/libunlook.so

# 2. Check library dependencies
ldd build/lib/libunlook.so
# Should show all dependencies resolved

# 3. Test basic functionality (mock mode)
cd /home/alessandro/unlook-standalone
./build/bin/unlook_scanner --help
# Should show usage information

# 4. Verify installation capability
sudo make install -C build
# Should install to system directories
```

---

**✅ INSTALLATION COMPLETE**

After running all commands above, your system will be ready to build and run the Unlook 3D Scanner Phase 1 system with full functionality.

**Next Steps:**
1. Run: `./build.sh --clean -j 4`
2. Verify build success
3. Test applications in `build/bin/`

**Support:** If any dependency installation fails, check the troubleshooting section above or verify you're running on Raspberry Pi OS (Debian-based).