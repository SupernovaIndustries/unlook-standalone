# Installation and packaging configuration

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set installation directories
set(UNLOOK_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(UNLOOK_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})
set(UNLOOK_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/unlook)
set(UNLOOK_INSTALL_DATADIR ${CMAKE_INSTALL_DATADIR}/unlook)
set(UNLOOK_INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/Unlook)

# Install directories for third-party libraries
set(UNLOOK_INSTALL_THIRD_PARTY_LIBDIR ${CMAKE_INSTALL_LIBDIR}/unlook/third-party)

# Version information
set(UNLOOK_VERSION ${PROJECT_VERSION})
set(UNLOOK_SOVERSION ${PROJECT_VERSION_MAJOR})

# Create version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/UnlookConfigVersion.cmake
    VERSION ${UNLOOK_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Create config file
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/UnlookConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/UnlookConfig.cmake
    INSTALL_DESTINATION ${UNLOOK_INSTALL_CONFIGDIR}
    PATH_VARS UNLOOK_INSTALL_INCLUDEDIR UNLOOK_INSTALL_LIBDIR
)

# Install config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/UnlookConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/UnlookConfigVersion.cmake
    DESTINATION ${UNLOOK_INSTALL_CONFIGDIR}
)

# Install third-party libraries that were built
if(OPENCV_BUILT)
    install(DIRECTORY ${THIRD_PARTY_INSTALL_DIR}/lib/
        DESTINATION ${UNLOOK_INSTALL_THIRD_PARTY_LIBDIR}
        FILES_MATCHING PATTERN "libopencv*.so*"
    )
    install(DIRECTORY ${THIRD_PARTY_INSTALL_DIR}/include/opencv4/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/third-party/opencv4
    )
endif()

if(LIBCAMERA_SYNC_BUILT)
    install(DIRECTORY ${THIRD_PARTY_INSTALL_DIR}/lib/
        DESTINATION ${UNLOOK_INSTALL_THIRD_PARTY_LIBDIR}
        FILES_MATCHING PATTERN "libcamera*.so*"
    )
    install(DIRECTORY ${THIRD_PARTY_INSTALL_DIR}/include/libcamera/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/third-party/libcamera
    )
endif()

if(JAVA_BUILT)
    install(DIRECTORY ${THIRD_PARTY_INSTALL_DIR}/java/
        DESTINATION ${UNLOOK_INSTALL_DATADIR}/java
        USE_SOURCE_PERMISSIONS
    )
endif()

# Desktop integration for GUI application
if(BUILD_GUI)
    # Desktop file
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/desktop/unlook-scanner.desktop.in
        ${CMAKE_CURRENT_BINARY_DIR}/unlook-scanner.desktop
        @ONLY
    )
    
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/unlook-scanner.desktop
        DESTINATION ${CMAKE_INSTALL_DATADIR}/applications
    )
    
    # Icon
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/unlook.png)
        install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/unlook.png
            DESTINATION ${CMAKE_INSTALL_DATADIR}/pixmaps
            RENAME unlook-scanner.png
        )
    endif()
    
    # Menu category
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/desktop/unlook-scanner.menu
        DESTINATION ${CMAKE_INSTALL_DATADIR}/menu
        OPTIONAL
    )
endif()

# Install calibration data and examples
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/calibration/
    DESTINATION ${UNLOOK_INSTALL_DATADIR}/calibration
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

# Install documentation
install(FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/README.md
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    DESTINATION ${UNLOOK_INSTALL_DATADIR}/doc
    OPTIONAL
)

# Development files installation
if(BUILD_SHARED_LIBS)
    # Export targets
    install(EXPORT UnlookTargets
        FILE UnlookTargets.cmake
        NAMESPACE Unlook::
        DESTINATION ${UNLOOK_INSTALL_CONFIGDIR}
    )
endif()

# Uninstall target
if(NOT TARGET uninstall)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
        IMMEDIATE @ONLY
    )

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
        COMMENT "Uninstalling Unlook Scanner"
    )
endif()

# Package configuration
set(CPACK_PACKAGE_NAME "unlook-scanner")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Professional 3D Scanner with 0.005mm precision")
set(CPACK_PACKAGE_DESCRIPTION "Unlook is a professional modular opensource 3D scanner designed for industrial, educational and professional applications.")
set(CPACK_PACKAGE_VENDOR "Unlook Project")
set(CPACK_PACKAGE_CONTACT "support@unlook.tech")

# Platform-specific packaging
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # DEB package for Debian/Ubuntu
    set(CPACK_GENERATOR "DEB;TGZ")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER ${CPACK_PACKAGE_CONTACT})
    set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
    set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/unlook/unlook-scanner")
    
    # Debian dependencies
    set(CPACK_DEBIAN_PACKAGE_DEPENDS 
        "libc6 (>= 2.17), libgcc1 (>= 1:4.2), libstdc++6 (>= 6)"
    )
    
    if(BUILD_GUI AND QT5_FOUND)
        set(CPACK_DEBIAN_PACKAGE_DEPENDS 
            "${CPACK_DEBIAN_PACKAGE_DEPENDS}, qtbase5 (>= 5.12)"
        )
    endif()
    
    # RPM package for RedHat/CentOS
    list(APPEND CPACK_GENERATOR "RPM")
    set(CPACK_RPM_PACKAGE_GROUP "Applications/Graphics")
    set(CPACK_RPM_PACKAGE_LICENSE "MIT")
    set(CPACK_RPM_PACKAGE_URL "https://github.com/unlook/unlook-scanner")
    
    # Architecture-specific naming
    if(IS_RASPBERRY_PI)
        set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-rpi-${CMAKE_SYSTEM_PROCESSOR}")
    else()
        set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()

# Include CPack
include(CPack)

# Custom packaging targets
add_custom_target(package-deb
    COMMAND ${CMAKE_CPACK_COMMAND} -G DEB
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Creating DEB package"
)

add_custom_target(package-rpm  
    COMMAND ${CMAKE_CPACK_COMMAND} -G RPM
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Creating RPM package"
)

# Installation summary
message(STATUS "")
message(STATUS "Installation Configuration:")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Binary directory: ${CMAKE_INSTALL_PREFIX}/${UNLOOK_INSTALL_BINDIR}")
message(STATUS "  Library directory: ${CMAKE_INSTALL_PREFIX}/${UNLOOK_INSTALL_LIBDIR}")
message(STATUS "  Include directory: ${CMAKE_INSTALL_PREFIX}/${UNLOOK_INSTALL_INCLUDEDIR}")
message(STATUS "  Data directory: ${CMAKE_INSTALL_PREFIX}/${UNLOOK_INSTALL_DATADIR}")
message(STATUS "")