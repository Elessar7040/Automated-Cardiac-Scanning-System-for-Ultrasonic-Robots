# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ur_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ur_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ur_FOUND FALSE)
  elseif(NOT ur_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ur_FOUND FALSE)
  endif()
  return()
endif()
set(_ur_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ur_FIND_QUIETLY)
  message(STATUS "Found ur: 0.3.0 (${ur_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ur' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ur_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ur_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ur_DIR}/${_extra}")
endforeach()