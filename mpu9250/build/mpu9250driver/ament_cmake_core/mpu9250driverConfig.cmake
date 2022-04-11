# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mpu9250driver_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mpu9250driver_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mpu9250driver_FOUND FALSE)
  elseif(NOT mpu9250driver_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mpu9250driver_FOUND FALSE)
  endif()
  return()
endif()
set(_mpu9250driver_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mpu9250driver_FIND_QUIETLY)
  message(STATUS "Found mpu9250driver: 0.0.1 (${mpu9250driver_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mpu9250driver' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mpu9250driver_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mpu9250driver_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mpu9250driver_DIR}/${_extra}")
endforeach()
