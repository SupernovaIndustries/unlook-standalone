# Dependency detection and configuration

# System package manager detection
find_program(APT_GET_CMD apt-get)
find_program(YUM_CMD yum)
find_program(DNF_CMD dnf)

# Essential system dependencies
find_package(PkgConfig REQUIRED)

# OpenCV - Critical for stereo vision
if(USE_SYSTEM_LIBS)
    find_package(OpenCV QUIET COMPONENTS core imgproc calib3d imgcodecs highgui)
    if(OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER_EQUAL "4.2.0")
        message(STATUS "Found system OpenCV ${OpenCV_VERSION}")
        set(OPENCV_FOUND TRUE)
    else()
        message(STATUS "System OpenCV not found or version too old, will build from source")
        set(OPENCV_FOUND FALSE)
    endif()
else()
    set(OPENCV_FOUND FALSE)
endif()

# Qt5 - For GUI application
if(BUILD_GUI)
    if(USE_SYSTEM_LIBS)
        find_package(Qt5 QUIET COMPONENTS Core Widgets Gui OpenGL)
        if(Qt5_FOUND)
            message(STATUS "Found system Qt5 ${Qt5_VERSION}")
            set(QT5_FOUND TRUE)
        else()
            message(STATUS "System Qt5 not found, will build from source")
            set(QT5_FOUND FALSE)
        endif()
    else()
        set(QT5_FOUND FALSE)
    endif()
endif()

# Eigen3 - Mathematical operations
if(USE_SYSTEM_LIBS)
    find_package(Eigen3 3.3 QUIET NO_MODULE)
    if(Eigen3_FOUND)
        message(STATUS "Found system Eigen3 ${Eigen3_VERSION}")
        set(EIGEN3_FOUND TRUE)
    else()
        message(STATUS "System Eigen3 not found, will use header-only version")
        set(EIGEN3_FOUND FALSE)
    endif()
else()
    set(EIGEN3_FOUND FALSE)
endif()

# libcamera-sync - Custom camera synchronization library
# First check system installation at /usr/local
find_path(LIBCAMERA_SYNC_INCLUDE_DIR 
    NAMES libcamera/libcamera.h
    PATHS /usr/local/include
    NO_DEFAULT_PATH
)

find_library(LIBCAMERA_SYNC_LIBRARY
    NAMES camera
    PATHS /usr/local/lib /usr/local/lib64
    NO_DEFAULT_PATH  
)

if(LIBCAMERA_SYNC_INCLUDE_DIR AND LIBCAMERA_SYNC_LIBRARY)
    message(STATUS "Found system libcamera-sync at /usr/local")
    set(LIBCAMERA_SYNC_FOUND TRUE)
    
    # Create imported target
    add_library(libcamera-sync::camera SHARED IMPORTED)
    set_target_properties(libcamera-sync::camera PROPERTIES
        IMPORTED_LOCATION ${LIBCAMERA_SYNC_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${LIBCAMERA_SYNC_INCLUDE_DIR}
    )
else()
    message(STATUS "System libcamera-sync not found, will build from third-party")
    set(LIBCAMERA_SYNC_FOUND FALSE)
endif()

# Java Runtime Environment - Required for BoofCV
find_package(Java 11 QUIET COMPONENTS Runtime Development)
if(Java_FOUND)
    message(STATUS "Found Java ${Java_VERSION}")
    set(JAVA_FOUND TRUE)
else()
    message(STATUS "Java not found, will install embedded OpenJDK")
    set(JAVA_FOUND FALSE)
endif()

# Check for essential build tools
find_program(NINJA_EXECUTABLE ninja)
if(NINJA_EXECUTABLE)
    message(STATUS "Found Ninja build system")
endif()

find_program(MESON_EXECUTABLE meson)
if(MESON_EXECUTABLE)
    message(STATUS "Found Meson build system")
endif()

find_program(GRADLE_EXECUTABLE gradle)
if(GRADLE_EXECUTABLE)
    message(STATUS "Found Gradle build system")
endif()

# Function to check system library versions
function(check_library_version LIB_NAME MIN_VERSION)
    pkg_check_modules(${LIB_NAME}_PC ${LIB_NAME}>=${MIN_VERSION} QUIET)
    if(${LIB_NAME}_PC_FOUND)
        message(STATUS "Found system ${LIB_NAME} ${${LIB_NAME}_PC_VERSION}")
        set(${LIB_NAME}_SYSTEM_OK TRUE PARENT_SCOPE)
    else()
        message(STATUS "System ${LIB_NAME} not found or version too old")
        set(${LIB_NAME}_SYSTEM_OK FALSE PARENT_SCOPE)
    endif()
endfunction()

# Create interface library for all dependencies
add_library(unlook_dependencies INTERFACE)

# Link system libraries that were found
if(OPENCV_FOUND)
    target_link_libraries(unlook_dependencies INTERFACE ${OpenCV_LIBS})
    target_include_directories(unlook_dependencies INTERFACE ${OpenCV_INCLUDE_DIRS})
endif()

if(QT5_FOUND)
    target_link_libraries(unlook_dependencies INTERFACE 
        Qt5::Core Qt5::Widgets Qt5::Gui Qt5::OpenGL)
endif()

if(EIGEN3_FOUND)
    target_link_libraries(unlook_dependencies INTERFACE Eigen3::Eigen)
endif()

if(LIBCAMERA_SYNC_FOUND)
    target_link_libraries(unlook_dependencies INTERFACE libcamera-sync::camera)
endif()

# Print dependency summary
message(STATUS "")
message(STATUS "Dependency Summary:")
message(STATUS "  OpenCV: ${OPENCV_FOUND}")
message(STATUS "  Qt5: ${QT5_FOUND}")
message(STATUS "  Eigen3: ${EIGEN3_FOUND}")
message(STATUS "  libcamera-sync: ${LIBCAMERA_SYNC_FOUND}")
message(STATUS "  Java: ${JAVA_FOUND}")
message(STATUS "")

# Check if we need to build any third-party dependencies
set(NEED_THIRD_PARTY FALSE)
if(NOT OPENCV_FOUND OR NOT QT5_FOUND OR NOT EIGEN3_FOUND OR NOT LIBCAMERA_SYNC_FOUND OR NOT JAVA_FOUND)
    set(NEED_THIRD_PARTY TRUE)
    message(STATUS "Some dependencies will be built from third-party sources")
endif()