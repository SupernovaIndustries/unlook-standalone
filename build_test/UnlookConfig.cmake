
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was UnlookConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Unlook 3D Scanner API CMake Configuration File

set(UNLOOK_VERSION 1.0.0)

# Find dependencies
find_dependency(OpenCV 4.0 REQUIRED COMPONENTS core imgproc imgcodecs calib3d features2d)
find_dependency(Threads REQUIRED)

# Find OpenMP if available
find_package(OpenMP QUIET)

# Include targets
if(NOT TARGET Unlook::unlook)
    include("${CMAKE_CURRENT_LIST_DIR}/UnlookTargets.cmake")
endif()

# Set variables for backward compatibility
set(UNLOOK_LIBRARIES Unlook::unlook)
set(UNLOOK_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")
set(UNLOOK_FOUND TRUE)

# Check required components
check_required_components(Unlook)

# Print status message
if(NOT Unlook_FIND_QUIETLY)
    message(STATUS "Found Unlook 3D Scanner API: ${UNLOOK_VERSION}")
    message(STATUS "  Libraries: ${UNLOOK_LIBRARIES}")
    message(STATUS "  Include dirs: ${UNLOOK_INCLUDE_DIRS}")
endif()

# Usage example:
# find_package(Unlook REQUIRED)
# target_link_libraries(your_target Unlook::unlook)
