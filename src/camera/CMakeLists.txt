# Camera implementation library
add_library(unlook_camera_impl STATIC
    camera_system.cpp
    CameraSystem.cpp
    CameraDevice.cpp
    CameraSynchronizer.cpp
    CameraUtils.cpp
    AutoExposure.cpp
    HardwareSyncManager.cpp
    LibcameraSyncDevice.cpp
    SynchronizedCameraSystem.cpp
    HardwareSyncCapture.cpp
    RawCameraInterface.cpp
)

target_include_directories(unlook_camera_impl
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(unlook_camera_impl
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    Threads::Threads
)

target_include_directories(unlook_camera_impl PRIVATE
    ${LIBCAMERA_INCLUDE_DIRS}
)

# Installation
install(TARGETS unlook_camera_impl
    EXPORT UnlookTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
)