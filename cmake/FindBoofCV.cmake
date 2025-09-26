#
# FindBoofCV.cmake - CMake module to find BoofCV Java library for stereo processing
#
# This module finds BoofCV JAR files and JNI components required for high-precision
# stereo vision processing in the Unlook 3D Scanner project.
#
# Variables set by this module:
#   BoofCV_FOUND          - System has BoofCV and JNI support
#   BoofCV_JAR_PATH       - Path to the BoofCV JAR file
#   BoofCV_CLASSPATH      - Java classpath for BoofCV
#   BoofCV_VERSION        - Version of BoofCV found
#   JNI_FOUND             - JNI development headers found
#   JNI_INCLUDE_DIRS      - JNI include directories
#   Java_FOUND            - Java runtime found
#   HAVE_BOOFCV           - Preprocessor definition for C++ code
#

cmake_minimum_required(VERSION 3.16)

# Find Java Runtime and Development Kit
find_package(Java COMPONENTS Runtime Development QUIET)

set(BoofCV_FOUND FALSE)
set(HAVE_BOOFCV FALSE)

if(NOT Java_FOUND)
    message(STATUS "BoofCV: Java Runtime not found - BoofCV integration disabled")
    return()
endif()

# Manual JNI detection for better compatibility
set(JNI_FOUND FALSE)
set(JNI_INCLUDE_DIRS "")

# Try to find JNI headers manually
find_path(JAVA_INCLUDE_PATH jni.h
    PATHS
    /usr/lib/jvm/java-17-openjdk-arm64/include
    /usr/lib/jvm/java-11-openjdk-arm64/include
    /usr/lib/jvm/java-8-openjdk-arm64/include
    /usr/lib/jvm/default-java/include
    /usr/lib/jvm/java-17-openjdk-amd64/include
    /usr/lib/jvm/java-11-openjdk-amd64/include
    /usr/lib/jvm/java-8-openjdk-amd64/include
    /System/Library/Frameworks/JavaVM.framework/Headers
    /opt/java/include
    NO_DEFAULT_PATH)

if(JAVA_INCLUDE_PATH)
    # Find platform-specific headers (jni_md.h)
    find_path(JAVA_INCLUDE_PATH2 jni_md.h
        PATHS
        ${JAVA_INCLUDE_PATH}/linux
        ${JAVA_INCLUDE_PATH}/darwin
        ${JAVA_INCLUDE_PATH}/win32
        NO_DEFAULT_PATH)

    if(JAVA_INCLUDE_PATH2)
        set(JNI_INCLUDE_DIRS ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})

        # Find JVM library for linking
        find_library(JVM_LIBRARY jvm
            PATHS
            /usr/lib/jvm/java-17-openjdk-arm64/lib/server
            /usr/lib/jvm/java-11-openjdk-arm64/lib/server
            /usr/lib/jvm/java-8-openjdk-arm64/lib/server
            /usr/lib/jvm/java-17-openjdk-amd64/lib/server
            /usr/lib/jvm/java-11-openjdk-amd64/lib/server
            /usr/lib/jvm/java-8-openjdk-amd64/lib/server
            /System/Library/Frameworks/JavaVM.framework/Libraries
            NO_DEFAULT_PATH)

        if(JVM_LIBRARY)
            set(JNI_LIBRARIES ${JVM_LIBRARY})
            set(JNI_FOUND TRUE)
            message(STATUS "BoofCV: JNI headers found at ${JAVA_INCLUDE_PATH}")
            message(STATUS "BoofCV: JVM library found at ${JVM_LIBRARY}")
        else()
            message(STATUS "BoofCV: JVM library (libjvm.so) not found")
        endif()
    else()
        message(STATUS "BoofCV: Platform-specific JNI headers (jni_md.h) not found")
    endif()
else()
    message(STATUS "BoofCV: JNI headers (jni.h) not found")
endif()

if(NOT JNI_FOUND)
    message(STATUS "BoofCV: JNI development libraries not found - BoofCV integration disabled")
    return()
endif()

# Define possible BoofCV JAR locations (in order of preference)
set(BOOFCV_SEARCH_PATHS
    # Third-party directory (project-specific)
    "${CMAKE_SOURCE_DIR}/third-party/BoofCV"
    "${CMAKE_SOURCE_DIR}/third-party/boofcv"
    "${CMAKE_SOURCE_DIR}/third-party"

    # System locations
    "/usr/share/java/boofcv"
    "/usr/local/share/java/boofcv"
    "/opt/boofcv"

    # User-specific locations
    "$ENV{HOME}/.local/share/java/boofcv"
    "$ENV{HOME}/boofcv"
)

# Define possible JAR file names (in order of preference)
set(BOOFCV_JAR_NAMES
    "unlook-boofcv-wrapper-1.1.6.jar"
    "boofcv-all-1.1.6.jar"
    "boofcv-all.jar"
    "boofcv-geo-1.1.6.jar"
    "boofcv-core-1.1.6.jar"
)

# Search for BoofCV JAR files
message(STATUS "BoofCV: Searching for JAR files...")
foreach(search_path ${BOOFCV_SEARCH_PATHS})
    if(EXISTS "${search_path}")
        message(STATUS "BoofCV: Checking ${search_path}")
        foreach(jar_name ${BOOFCV_JAR_NAMES})
            set(jar_full_path "${search_path}/${jar_name}")
            if(EXISTS "${jar_full_path}")
                message(STATUS "BoofCV: Found ${jar_name} at ${jar_full_path}")

                # Check file size (should be > 1KB for a real JAR)
                file(SIZE "${jar_full_path}" jar_size)
                if(jar_size GREATER 1024)
                    list(APPEND BoofCV_JARS "${jar_full_path}")
                    message(STATUS "BoofCV: JAR file size: ${jar_size} bytes (valid)")
                else()
                    message(STATUS "BoofCV: JAR file size: ${jar_size} bytes (too small, skipping)")
                endif()
            endif()
        endforeach()
    endif()
endforeach()

# If we found JAR files, set up the integration
if(BoofCV_JARS)
    # Use the first (best) JAR found
    list(GET BoofCV_JARS 0 BoofCV_JAR_PATH)

    # Create classpath with all found JARs
    string(REPLACE ";" ":" BoofCV_CLASSPATH "${BoofCV_JARS}")

    # Extract version from JAR filename
    get_filename_component(jar_filename "${BoofCV_JAR_PATH}" NAME)
    if(jar_filename MATCHES ".*-([0-9]+\\.[0-9]+\\.[0-9]+)\\.jar")
        set(BoofCV_VERSION "${CMAKE_MATCH_1}")
    else()
        set(BoofCV_VERSION "unknown")
    endif()

    # Test if we can actually use the JAR with Java
    message(STATUS "BoofCV: Testing JAR file accessibility...")
    execute_process(
        COMMAND "${Java_JAVA_EXECUTABLE}" -cp "${BoofCV_JAR_PATH}" -version
        RESULT_VARIABLE java_test_result
        OUTPUT_QUIET
        ERROR_QUIET
    )

    if(java_test_result EQUAL 0)
        set(BoofCV_FOUND TRUE)
        set(HAVE_BOOFCV TRUE)

        message(STATUS "BoofCV: Integration enabled")
        message(STATUS "BoofCV: Primary JAR: ${BoofCV_JAR_PATH}")
        message(STATUS "BoofCV: Classpath: ${BoofCV_CLASSPATH}")
        message(STATUS "BoofCV: Version: ${BoofCV_VERSION}")
        message(STATUS "BoofCV: Java Runtime: ${Java_JAVA_EXECUTABLE} (${Java_VERSION})")
        message(STATUS "BoofCV: JNI Headers: ${JNI_INCLUDE_DIRS}")
    else()
        message(WARNING "BoofCV: JAR file found but not accessible with Java - integration disabled")
    endif()
else()
    message(STATUS "BoofCV: No suitable JAR files found - integration disabled")
    message(STATUS "BoofCV: Searched paths: ${BOOFCV_SEARCH_PATHS}")
    message(STATUS "BoofCV: Searched names: ${BOOFCV_JAR_NAMES}")
endif()

# Create target for BoofCV integration if found
if(BoofCV_FOUND)
    add_library(BoofCV::BoofCV INTERFACE IMPORTED)

    # Set target properties
    set_target_properties(BoofCV::BoofCV PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${JNI_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${JNI_LIBRARIES}"
        INTERFACE_COMPILE_DEFINITIONS "HAVE_BOOFCV=1;BOOFCV_JAR_PATH=\"${BoofCV_JAR_PATH}\""
    )

    # Create cache variables for user configuration
    set(BOOFCV_JAR_PATH "${BoofCV_JAR_PATH}" CACHE STRING "Path to BoofCV JAR file")
    set(BOOFCV_CLASSPATH "${BoofCV_CLASSPATH}" CACHE STRING "BoofCV Java classpath")
    set(BOOFCV_JNI_LIBRARIES "${JNI_LIBRARIES}" CACHE STRING "JNI libraries for BoofCV")

    mark_as_advanced(BOOFCV_JAR_PATH BOOFCV_CLASSPATH BOOFCV_JNI_LIBRARIES)
endif()

# Print final summary
if(BoofCV_FOUND)
    message(STATUS "BoofCV: ✓ ENABLED - High-precision stereo processing available")
else()
    message(STATUS "BoofCV: ✗ DISABLED - Falling back to OpenCV stereo algorithms")
endif()