#!/bin/bash

# BoofCV Download and Build Script
# Downloads BoofCV source and builds the Java library

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
THIRD_PARTY_DIR="${PROJECT_ROOT}/third-party"
BOOFCV_DIR="${THIRD_PARTY_DIR}/BoofCV"
INSTALL_DIR="${THIRD_PARTY_DIR}/install/BoofCV"

BOOFCV_VERSION="v0.43"
BOOFCV_URL="https://github.com/lessthanoptimal/BoofCV.git"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() { echo -e "${BLUE}[BoofCV]${NC} $*"; }
log_success() { echo -e "${GREEN}[BoofCV]${NC} $*"; }
log_warning() { echo -e "${YELLOW}[BoofCV]${NC} $*"; }
log_error() { echo -e "${RED}[BoofCV]${NC} $*" >&2; }

# Check if Java is available
check_java() {
    if ! command -v java >/dev/null 2>&1; then
        log_error "Java not found. BoofCV requires Java 11 or later."
        
        # Try embedded Java from third-party
        local java_home="${THIRD_PARTY_DIR}/install/java"
        if [[ -d "$java_home" ]] && [[ -x "$java_home/bin/java" ]]; then
            export JAVA_HOME="$java_home"
            export PATH="$java_home/bin:$PATH"
            log_info "Using embedded Java from $java_home"
        else
            log_error "Please install Java 11+ or run build script with --deps to install embedded Java"
            return 1
        fi
    fi
    
    # Check Java version
    local java_version=$(java -version 2>&1 | head -n1 | grep -oE '[0-9]+' | head -n1)
    if [[ $java_version -lt 11 ]]; then
        log_error "Java version $java_version is too old. BoofCV requires Java 11 or later."
        return 1
    fi
    
    log_success "Found Java $(java -version 2>&1 | head -n1)"
}

# Check if Gradle is available
check_gradle() {
    if ! command -v gradle >/dev/null 2>&1; then
        log_info "Gradle not found, will use Gradle wrapper from BoofCV"
        USE_GRADLE_WRAPPER=true
    else
        log_success "Found Gradle $(gradle --version | head -n1)"
        USE_GRADLE_WRAPPER=false
    fi
}

# Download BoofCV source
download_boofcv() {
    log_info "Downloading BoofCV $BOOFCV_VERSION..."
    
    if [[ -d "$BOOFCV_DIR" ]]; then
        log_info "BoofCV directory exists, updating..."
        cd "$BOOFCV_DIR"
        git fetch --all
        git checkout "$BOOFCV_VERSION"
        git pull origin "$BOOFCV_VERSION" || true
    else
        git clone --branch "$BOOFCV_VERSION" --depth 1 "$BOOFCV_URL" "$BOOFCV_DIR"
    fi
    
    log_success "BoofCV source downloaded"
}

# Build BoofCV
build_boofcv() {
    log_info "Building BoofCV..."
    
    cd "$BOOFCV_DIR"
    
    # Make sure we have the right permissions
    if [[ -f "./gradlew" ]]; then
        chmod +x "./gradlew"
    fi
    
    # Choose build command
    local gradle_cmd
    if [[ "$USE_GRADLE_WRAPPER" == true ]] && [[ -f "./gradlew" ]]; then
        gradle_cmd="./gradlew"
    elif command -v gradle >/dev/null 2>&1; then
        gradle_cmd="gradle"
    else
        log_error "Neither Gradle nor Gradle wrapper found"
        return 1
    fi
    
    # Build the jar files
    log_info "Running Gradle build..."
    $gradle_cmd assemble
    
    # Create installation directory
    mkdir -p "$INSTALL_DIR/lib"
    mkdir -p "$INSTALL_DIR/include"
    
    # Copy built JAR files
    log_info "Installing BoofCV libraries..."
    
    # Find and copy core BoofCV jars
    find . -name "*.jar" -path "*/libs/*" -not -path "*/test*" | while read -r jar_file; do
        local jar_name=$(basename "$jar_file")
        log_info "Installing $jar_name"
        cp "$jar_file" "$INSTALL_DIR/lib/"
    done
    
    # Create a comprehensive classpath file for easy linking
    echo "# BoofCV Classpath" > "$INSTALL_DIR/boofcv_classpath.txt"
    find "$INSTALL_DIR/lib" -name "*.jar" | tr '\n' ':' >> "$INSTALL_DIR/boofcv_classpath.txt"
    
    # Create version file
    echo "$BOOFCV_VERSION" > "$INSTALL_DIR/VERSION"
    
    log_success "BoofCV built and installed to $INSTALL_DIR"
}

# Create BoofCV configuration
create_config() {
    local config_file="$INSTALL_DIR/boofcv_config.cmake"
    
    log_info "Creating CMake configuration..."
    
    cat > "$config_file" << EOF
# BoofCV CMake Configuration
# Generated automatically by download_boofcv.sh

set(BOOFCV_FOUND TRUE)
set(BOOFCV_VERSION "$BOOFCV_VERSION")
set(BOOFCV_INSTALL_DIR "$INSTALL_DIR")

# Java classpath with all BoofCV jars
file(GLOB BOOFCV_JAR_FILES "\${BOOFCV_INSTALL_DIR}/lib/*.jar")
string(REPLACE ";" ":" BOOFCV_CLASSPATH "\${BOOFCV_JAR_FILES}")

# Function to add BoofCV to a target
function(target_link_boofcv TARGET_NAME)
    # Add Java dependency
    find_package(Java REQUIRED COMPONENTS Runtime Development)
    find_package(JNI REQUIRED)
    
    target_include_directories(\${TARGET_NAME} PRIVATE \${JNI_INCLUDE_DIRS})
    target_link_libraries(\${TARGET_NAME} PRIVATE \${JNI_LIBRARIES})
    
    # Set classpath for runtime
    target_compile_definitions(\${TARGET_NAME} PRIVATE 
        BOOFCV_CLASSPATH="\${BOOFCV_CLASSPATH}"
        BOOFCV_INSTALL_DIR="\${BOOFCV_INSTALL_DIR}"
    )
endfunction()

message(STATUS "Found BoofCV \${BOOFCV_VERSION} at \${BOOFCV_INSTALL_DIR}")
EOF
    
    log_success "CMake configuration created"
}

# Validate installation
validate_installation() {
    log_info "Validating BoofCV installation..."
    
    # Check if main jars exist
    local core_jars=("boofcv-core" "boofcv-ip" "boofcv-feature" "boofcv-geo" "boofcv-calibration")
    local missing_jars=()
    
    for jar_name in "${core_jars[@]}"; do
        if ! find "$INSTALL_DIR/lib" -name "${jar_name}*.jar" | grep -q .; then
            missing_jars+=("$jar_name")
        fi
    done
    
    if [[ ${#missing_jars[@]} -gt 0 ]]; then
        log_warning "Some expected jars not found: ${missing_jars[*]}"
        log_info "This might be due to BoofCV version differences"
    fi
    
    # List installed jars
    log_info "Installed BoofCV jars:"
    find "$INSTALL_DIR/lib" -name "*.jar" -exec basename {} \; | sort
    
    log_success "BoofCV validation completed"
}

# Main execution
main() {
    log_info "BoofCV Download and Build Script"
    log_info "================================="
    
    check_java
    check_gradle
    download_boofcv
    build_boofcv
    create_config
    validate_installation
    
    log_success "BoofCV setup completed successfully!"
    log_info "Installation directory: $INSTALL_DIR"
    log_info "Use target_link_boofcv() in CMake to add BoofCV to your targets"
}

# Execute main function
main "$@"