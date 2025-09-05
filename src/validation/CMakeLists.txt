# Validation library
add_library(unlook_validation STATIC
    CalibrationValidator.cpp
    DepthValidator.cpp
    EpipolarValidator.cpp
)

# Set target properties
set_target_properties(unlook_validation PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(unlook_validation 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(unlook_validation
    PUBLIC
        ${OpenCV_LIBS}
        unlook_calibration
)

# Installation and export
install(TARGETS unlook_validation
    EXPORT UnlookTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
)