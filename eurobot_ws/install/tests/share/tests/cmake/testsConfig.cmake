# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tests_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tests_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tests_FOUND FALSE)
  elseif(NOT tests_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tests_FOUND FALSE)
  endif()
  return()
endif()
set(_tests_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tests_FIND_QUIETLY)
  message(STATUS "Found tests: 0.0.0 (${tests_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tests' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tests_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tests_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tests_DIR}/${_extra}")
endforeach()