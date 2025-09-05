#!/bin/bash

# BoofCV Setup Script for Unlook Stereo Calibration System
# Downloads and configures BoofCV and BoofCPP for high-precision calibration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
THIRD_PARTY_DIR="$PROJECT_ROOT/third-party"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== BoofCV Setup for Unlook 3D Scanner ===${NC}"

# Create third-party directory
mkdir -p "$THIRD_PARTY_DIR"
cd "$THIRD_PARTY_DIR"

# Check for Java
echo -e "${YELLOW}Checking Java installation...${NC}"
if ! command -v java &> /dev/null; then
    echo -e "${RED}Java is not installed. Installing OpenJDK...${NC}"
    sudo apt-get update
    sudo apt-get install -y openjdk-11-jdk-headless
fi

java -version

# Download BoofCV
BOOFCV_VERSION="0.43"
BOOFCV_DIR="$THIRD_PARTY_DIR/BoofCV"

if [ ! -d "$BOOFCV_DIR" ]; then
    echo -e "${YELLOW}Downloading BoofCV v${BOOFCV_VERSION}...${NC}"
    git clone https://github.com/lessthanoptimal/BoofCV.git
    cd BoofCV
    git checkout v${BOOFCV_VERSION}
    
    # Build BoofCV
    echo -e "${YELLOW}Building BoofCV...${NC}"
    ./gradlew assemble
    
    # Build calibration module specifically
    ./gradlew :applications:assemble
    cd ..
else
    echo -e "${GREEN}BoofCV already exists${NC}"
fi

# Download BoofCPP
BOOFCPP_DIR="$THIRD_PARTY_DIR/BoofCPP"

if [ ! -d "$BOOFCPP_DIR" ]; then
    echo -e "${YELLOW}Downloading BoofCPP...${NC}"
    git clone https://github.com/lessthanoptimal/BoofCPP.git
    cd BoofCPP
    
    # Create build directory
    mkdir -p build
    cd build
    
    # Configure with CMake
    echo -e "${YELLOW}Configuring BoofCPP...${NC}"
    cmake .. -DCMAKE_BUILD_TYPE=Release
    
    # Build
    echo -e "${YELLOW}Building BoofCPP...${NC}"
    make -j$(nproc)
    
    cd ../..
else
    echo -e "${GREEN}BoofCPP already exists${NC}"
fi

# Create JNI wrapper directory
JNI_DIR="$THIRD_PARTY_DIR/boofcv-jni"
mkdir -p "$JNI_DIR"

# Create CMake configuration for BoofCV integration
cat > "$THIRD_PARTY_DIR/BoofCVConfig.cmake" << 'EOF'
# BoofCV Configuration for CMake

set(BOOFCV_FOUND TRUE)
set(BOOFCV_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(BOOFCV_INCLUDE_DIRS "${BOOFCV_ROOT_DIR}/BoofCPP/include")
set(BOOFCV_LIBRARY_DIRS "${BOOFCV_ROOT_DIR}/BoofCPP/build/lib")
set(BOOFCV_LIBRARIES "boofcpp")

# Java paths for JNI
find_package(JNI REQUIRED)
if(JNI_FOUND)
    set(BOOFCV_JNI_INCLUDE_DIRS ${JNI_INCLUDE_DIRS})
endif()

# BoofCV JAR files
file(GLOB BOOFCV_JARS "${BOOFCV_ROOT_DIR}/BoofCV/applications/build/libs/*.jar")

# Function to setup BoofCV for a target
function(target_link_boofcv target)
    target_include_directories(${target} PRIVATE ${BOOFCV_INCLUDE_DIRS} ${BOOFCV_JNI_INCLUDE_DIRS})
    target_link_directories(${target} PRIVATE ${BOOFCV_LIBRARY_DIRS})
    target_link_libraries(${target} ${BOOFCV_LIBRARIES} ${JNI_LIBRARIES})
endfunction()
EOF

# Create setup completion marker
touch "$THIRD_PARTY_DIR/.boofcv_setup_complete"

echo -e "${GREEN}=== BoofCV Setup Complete ===${NC}"
echo -e "${GREEN}BoofCV and BoofCPP are ready for integration${NC}"
echo -e "${YELLOW}To use in CMake, add: find_package(BoofCV REQUIRED)${NC}"