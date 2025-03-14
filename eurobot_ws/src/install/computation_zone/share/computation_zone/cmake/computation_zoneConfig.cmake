# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_computation_zone_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED computation_zone_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(computation_zone_FOUND FALSE)
  elseif(NOT computation_zone_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(computation_zone_FOUND FALSE)
  endif()
  return()
endif()
set(_computation_zone_CONFIG_INCLUDED TRUE)

# output package information
if(NOT computation_zone_FIND_QUIETLY)
  message(STATUS "Found computation_zone: 0.0.0 (${computation_zone_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'computation_zone' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${computation_zone_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(computation_zone_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${computation_zone_DIR}/${_extra}")
endforeach()
