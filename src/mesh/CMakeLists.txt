# Mesh processing library for industrial mesh generation and export
# Provides comprehensive mesh validation, optimization, and export functionality

set(MESH_SOURCES
    MeshValidator.cpp
    MeshOptimizer.cpp
    MeshCleaner.cpp
    IndustrialMeshExporter.cpp
    PoissonReconstructor.cpp
)

set(MESH_HEADERS
    ${CMAKE_SOURCE_DIR}/include/unlook/mesh/MeshValidator.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/mesh/MeshOptimizer.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/mesh/MeshCleaner.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/mesh/IndustrialMeshExporter.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/mesh/PoissonReconstructor.hpp
)

# Create mesh library
add_library(unlook_mesh ${MESH_SOURCES})

# Set target properties
set_target_properties(unlook_mesh PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    OUTPUT_NAME "unlook_mesh"
    CLEAN_DIRECT_OUTPUT 1
)

# Include directories
target_include_directories(unlook_mesh
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

# Link required libraries
target_link_libraries(unlook_mesh
    PUBLIC
        ${OpenCV_LIBS}
        Threads::Threads
    PRIVATE
        ${FILESYSTEM_LIB}
)

# Add Open3D support if available
if(OPEN3D_ENABLED AND Open3D_LIBRARIES)
    target_link_libraries(unlook_mesh PRIVATE ${Open3D_LIBRARIES})
    target_compile_definitions(unlook_mesh PRIVATE OPEN3D_ENABLED=1)
    message(STATUS "Mesh: Open3D support enabled")
else()
    message(STATUS "Mesh: Using fallback implementations (no Open3D)")
endif()

# Add OpenMP support for parallel processing
if(OpenMP_CXX_FOUND)
    target_link_libraries(unlook_mesh PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(unlook_mesh PRIVATE HAVE_OPENMP=1)
    message(STATUS "Mesh: OpenMP acceleration enabled")
endif()

# ARM64 optimizations for CM4/CM5
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_compile_definitions(unlook_mesh PRIVATE ARM64_OPTIMIZED=1)
    message(STATUS "Mesh: ARM64 optimizations enabled")
endif()

# Compiler-specific optimizations
target_compile_options(unlook_mesh PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -pedantic>
    $<$<CXX_COMPILER_ID:GNU,Clang>:$<$<CONFIG:Debug>:-g -O0>>
    $<$<CXX_COMPILER_ID:GNU,Clang>:$<$<CONFIG:Release>:-O3 -DNDEBUG -march=native>>
)

# Industrial precision compile definitions
target_compile_definitions(unlook_mesh PRIVATE
    UNLOOK_MESH_PRECISION=0.005  # 5 micron target precision
    UNLOOK_MESH_INDUSTRIAL=1     # Enable industrial features
)

# Position Independent Code for shared library
if(BUILD_SHARED_LIBS)
    set_target_properties(unlook_mesh PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# Installation
install(TARGETS unlook_mesh
    EXPORT UnlookTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Runtime
        NAMELINK_COMPONENT Development
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT Runtime
)

# Install headers
install(FILES ${MESH_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/mesh
    COMPONENT Development
)

# Debug information
message(STATUS "Mesh library configuration:")
message(STATUS "  Sources: ${MESH_SOURCES}")
message(STATUS "  Headers: ${MESH_HEADERS}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")