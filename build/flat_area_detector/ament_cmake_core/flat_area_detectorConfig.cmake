# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_flat_area_detector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED flat_area_detector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(flat_area_detector_FOUND FALSE)
  elseif(NOT flat_area_detector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(flat_area_detector_FOUND FALSE)
  endif()
  return()
endif()
set(_flat_area_detector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT flat_area_detector_FIND_QUIETLY)
  message(STATUS "Found flat_area_detector: 0.1.0 (${flat_area_detector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'flat_area_detector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${flat_area_detector_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(flat_area_detector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${flat_area_detector_DIR}/${_extra}")
endforeach()
