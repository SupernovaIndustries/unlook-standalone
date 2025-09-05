# Camera library
add_library(unlook_camera STATIC
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
)

target_include_directories(unlook_camera PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_link_libraries(unlook_camera
    unlook_core
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

target_include_directories(unlook_camera PRIVATE
    ${LIBCAMERA_INCLUDE_DIRS}
)