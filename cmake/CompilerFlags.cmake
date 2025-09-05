# Compiler flags and optimization settings

# Set position independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Common compiler flags
set(COMMON_CXX_FLAGS 
    -Wall 
    -Wextra 
    -Wpedantic 
    -Wno-unused-parameter
    -fvisibility=hidden
)

# Debug flags
set(DEBUG_CXX_FLAGS 
    -g 
    -O0 
    -DDEBUG 
    -fno-omit-frame-pointer
    -fsanitize=address
    -fsanitize=undefined
)

# Release flags
set(RELEASE_CXX_FLAGS 
    -O3 
    -DNDEBUG 
    -fomit-frame-pointer
    -ffast-math
)

# ARM64/Raspberry Pi specific optimizations
if(ENABLE_ARM_OPTIMIZATIONS)
    list(APPEND RELEASE_CXX_FLAGS
        -march=armv8-a+crc
        -mtune=cortex-a72
        -ftree-vectorize
        -funsafe-math-optimizations
    )
    
    # Check for NEON support
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-mfpu=neon" COMPILER_SUPPORTS_NEON)
    if(COMPILER_SUPPORTS_NEON)
        list(APPEND RELEASE_CXX_FLAGS -mfpu=neon)
    endif()
    
    message(STATUS "ARM64 optimizations enabled")
endif()

# Link Time Optimization
if(ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT LTO_SUPPORTED OUTPUT LTO_ERROR)
    if(LTO_SUPPORTED)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
        message(STATUS "Link Time Optimization enabled")
    else()
        message(WARNING "LTO not supported: ${LTO_ERROR}")
    endif()
endif()

# Apply flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(compiler_flags INTERFACE ${COMMON_CXX_FLAGS} ${DEBUG_CXX_FLAGS})
    message(STATUS "Debug build flags applied")
else()
    target_compile_options(compiler_flags INTERFACE ${COMMON_CXX_FLAGS} ${RELEASE_CXX_FLAGS})
    message(STATUS "Release build flags applied")
endif()

# Create interface library for compiler flags
add_library(compiler_flags INTERFACE)

# Apply common flags
target_compile_options(compiler_flags INTERFACE ${COMMON_CXX_FLAGS})

# Apply build-specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(compiler_flags INTERFACE ${DEBUG_CXX_FLAGS})
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_link_options(compiler_flags INTERFACE -fsanitize=address -fsanitize=undefined)
    endif()
else()
    target_compile_options(compiler_flags INTERFACE ${RELEASE_CXX_FLAGS})
endif()

# Memory optimization for Raspberry Pi CM4 (8GB limit)
if(IS_RASPBERRY_PI)
    target_compile_definitions(compiler_flags INTERFACE 
        -DUNLOOK_MEMORY_OPTIMIZED=1
        -DUNLOOK_MAX_IMAGE_CACHE_MB=512
    )
    message(STATUS "Memory optimizations for Raspberry Pi enabled")
endif()

# Threading support
find_package(Threads REQUIRED)
target_link_libraries(compiler_flags INTERFACE Threads::Threads)