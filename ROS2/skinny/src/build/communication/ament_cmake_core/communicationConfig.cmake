# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_communication_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED communication_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(communication_FOUND FALSE)
  elseif(NOT communication_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(communication_FOUND FALSE)
  endif()
  return()
endif()
set(_communication_CONFIG_INCLUDED TRUE)

# output package information
if(NOT communication_FIND_QUIETLY)
  message(STATUS "Found communication: 1.0.0 (${communication_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'communication' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(communication_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${communication_DIR}/${_extra}")
endforeach()
