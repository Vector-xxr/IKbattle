# Compatibility module for CoIKS: find_package(CASADI REQUIRED)
# CasADi itself exports package name "casadi".

find_package(casadi CONFIG QUIET)
if(casadi_FOUND)
  set(CASADI_FOUND TRUE)
  if(NOT TARGET CASADI::casadi AND TARGET casadi::casadi)
    add_library(CASADI::casadi ALIAS casadi::casadi)
  endif()
  if(DEFINED casadi_INCLUDE_DIRS)
    set(CASADI_INCLUDE_DIRS ${casadi_INCLUDE_DIRS})
  endif()
  if(DEFINED casadi_LIBRARIES)
    set(CASADI_LIBRARIES ${casadi_LIBRARIES})
  endif()
else()
  find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp
    HINTS /usr/local/include /usr/include)
  find_library(CASADI_LIBRARY casadi
    HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(CASADI
    REQUIRED_VARS CASADI_LIBRARY CASADI_INCLUDE_DIR)
  if(CASADI_FOUND)
    set(CASADI_INCLUDE_DIRS ${CASADI_INCLUDE_DIR})
    set(CASADI_LIBRARIES ${CASADI_LIBRARY})
  endif()
endif()

mark_as_advanced(CASADI_INCLUDE_DIR CASADI_LIBRARY)
