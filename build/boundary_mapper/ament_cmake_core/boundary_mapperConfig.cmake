# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_boundary_mapper_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED boundary_mapper_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(boundary_mapper_FOUND FALSE)
  elseif(NOT boundary_mapper_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(boundary_mapper_FOUND FALSE)
  endif()
  return()
endif()
set(_boundary_mapper_CONFIG_INCLUDED TRUE)

# output package information
if(NOT boundary_mapper_FIND_QUIETLY)
  message(STATUS "Found boundary_mapper: 0.0.1 (${boundary_mapper_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'boundary_mapper' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${boundary_mapper_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(boundary_mapper_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${boundary_mapper_DIR}/${_extra}")
endforeach()
