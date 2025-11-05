# Core module - fundamental functionality

set(CORE_SOURCES
    Logger.cpp
    config.cpp
    exception.cpp
    Timer.cpp
)

set(CORE_HEADERS
    logger.h
    config.h
    exception.h
    timer.h
    types.h
)

# Core module as STATIC library for export compatibility
add_library(unlook_core STATIC ${CORE_SOURCES})

target_include_directories(unlook_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)

# Position Independent Code for shared library linkage
if(BUILD_SHARED_LIBS)
    set_target_properties(unlook_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# Installation and export
install(TARGETS unlook_core
    EXPORT UnlookTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
)

# Install headers
install(FILES ${CORE_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/core
    COMPONENT Development
)