# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gimbal_status_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gimbal_status_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gimbal_status_FOUND FALSE)
  elseif(NOT gimbal_status_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gimbal_status_FOUND FALSE)
  endif()
  return()
endif()
set(_gimbal_status_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gimbal_status_FIND_QUIETLY)
  message(STATUS "Found gimbal_status: 0.0.0 (${gimbal_status_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gimbal_status' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gimbal_status_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gimbal_status_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gimbal_status_DIR}/${_extra}")
endforeach()
