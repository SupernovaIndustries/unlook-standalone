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

# Core module can be built as separate library if needed
add_library(unlook_core OBJECT ${CORE_SOURCES})

target_include_directories(unlook_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(unlook_core PUBLIC compiler_flags)

# Install headers
install(FILES ${CORE_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/core
    COMPONENT Development
)