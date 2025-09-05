# ARM64/AArch64 Cross-compilation Toolchain
# For Raspberry Pi CM4/CM5 and other ARM64 targets

# Target system configuration
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_SYSTEM_VERSION 1)

# Cross-compilation tools
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_ASM_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_STRIP aarch64-linux-gnu-strip)
set(CMAKE_AR aarch64-linux-gnu-ar)
set(CMAKE_RANLIB aarch64-linux-gnu-ranlib)
set(CMAKE_OBJDUMP aarch64-linux-gnu-objdump)
set(CMAKE_OBJCOPY aarch64-linux-gnu-objcopy)

# Pkg-config for cross-compilation
set(PKG_CONFIG_EXECUTABLE aarch64-linux-gnu-pkg-config)

# Root filesystem paths (adjust for your sysroot if available)
# set(CMAKE_SYSROOT /path/to/aarch64-sysroot)
# set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Search paths configuration
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ARM64 specific compiler flags
set(ARM64_FLAGS "-march=armv8-a+crc -mtune=cortex-a72")

# Raspberry Pi CM4 optimizations
if(DEFINED CMAKE_BUILD_TYPE AND CMAKE_BUILD_TYPE STREQUAL "Release")
    set(ARM64_FLAGS "${ARM64_FLAGS} -O3 -ftree-vectorize -funsafe-math-optimizations")
endif()

# NEON SIMD optimizations
set(ARM64_FLAGS "${ARM64_FLAGS} -mfpu=neon-fp-armv8 -funsafe-math-optimizations")

# Apply flags
set(CMAKE_C_FLAGS_INIT "${ARM64_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${ARM64_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${ARM64_FLAGS}")

# Memory optimization for Raspberry Pi CM4 (8GB limit)
set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -DUNLOOK_MEMORY_OPTIMIZED=1")
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} -DUNLOOK_MEMORY_OPTIMIZED=1")

# Thread support
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)

# Position Independent Code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Cache variables to speed up configuration
set(CMAKE_C_COMPILER_WORKS TRUE CACHE BOOL "")
set(CMAKE_CXX_COMPILER_WORKS TRUE CACHE BOOL "")

# Raspberry Pi specific definitions
add_definitions(-DUNLOOK_RASPBERRY_PI=1)
add_definitions(-DUNLOOK_ARM64=1)
add_definitions(-DUNLOOK_CROSS_COMPILE=1)

# Inform about toolchain usage
message(STATUS "Using ARM64 cross-compilation toolchain")
message(STATUS "Target: ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "ARM64 Flags: ${ARM64_FLAGS}")

# Helper function to check if we're cross-compiling
function(check_cross_compile_setup)
    # Verify cross-compiler is available
    find_program(CROSS_GCC ${CMAKE_C_COMPILER})
    if(NOT CROSS_GCC)
        message(FATAL_ERROR 
            "Cross-compiler not found: ${CMAKE_C_COMPILER}\n"
            "Install with: sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu"
        )
    endif()
    
    # Verify target can be compiled for
    try_compile(CROSS_COMPILE_TEST
        ${CMAKE_BINARY_DIR}/cross_compile_test
        ${CMAKE_CURRENT_LIST_DIR}/test_cross_compile.c
    )
    
    if(NOT CROSS_COMPILE_TEST)
        message(WARNING "Cross-compilation test failed - check toolchain setup")
    else()
        message(STATUS "Cross-compilation test passed")
    endif()
endfunction()

# Call setup check
check_cross_compile_setup()