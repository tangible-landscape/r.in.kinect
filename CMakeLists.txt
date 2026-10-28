cmake_minimum_required(VERSION 3.22)

project(r.in.kinect LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If build_addon is not yet in module path (standalone build), locate GRASS first
if(NOT COMMAND build_addon)
  find_package(GRASS REQUIRED)
  if(GRASS_CMAKE_DIR)
    list(APPEND CMAKE_MODULE_PATH "${GRASS_CMAKE_DIR}")
  endif()
endif()

include(build_addon)

find_M()
find_OpenMP()

find_package(PCL REQUIRED COMPONENTS
  common
  io
  octree
  filters
  surface
  segmentation
  sample_consensus
  kdtree
  search
  features
  keypoints
  tracking)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(freenect2 REQUIRED)

# Wrap library lists into imported INTERFACE targets required by build_addon()
add_library(PCL_ALL INTERFACE IMPORTED)
set_target_properties(PCL_ALL PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES      "${PCL_LIBRARIES}"
  INTERFACE_COMPILE_DEFINITIONS "${PCL_DEFINITIONS}")

add_library(OpenCV_ALL INTERFACE IMPORTED)
set_target_properties(OpenCV_ALL PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES      "${OpenCV_LIBRARIES}")

add_library(FREENECT2_ALL INTERFACE IMPORTED)
set_target_properties(FREENECT2_ALL PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${freenect2_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES      "${freenect2_LIBRARIES}")

set(sources
  main.cpp
  drawing.cpp
  interp.cpp
  analyses.h
  binning.h
  binning_color.h
  calibrate.h
  drawing.h
  interp.h
  k2g.h)

# Check if running inside active GRASS session with documentation generator support
if(NOT DEFINED ENV{GISRC} AND NOT DEFINED ENV{LOCATION_NAME})
  set(_ADDON_DOC_ARGS NO_DOCS)
else()
  set(_ADDON_DOC_ARGS DOCFILES ${PROJECT_NAME}.html)
endif()

build_addon(
  ${PROJECT_NAME}
  SOURCES ${sources}
  GRASSLIBS
    gis
    raster
    imagery
    vector
    interpfl
    interpdata
    gmath
    qtree
    dbmiclient
  DEPENDS
    PCL_ALL
    OpenCV_ALL
    Eigen3::Eigen
    FREENECT2_ALL
    LIBM
  OPTIONAL_DEPENDS
    OpenMP::OpenMP_CXX
  ${_ADDON_DOC_ARGS})

# For standalone builds outside of an active GRASS session, install documentation directly
if(NOT DEFINED ENV{GISRC} AND NOT DEFINED ENV{LOCATION_NAME})
  install(FILES ${PROJECT_NAME}.html DESTINATION docs/html OPTIONAL)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wno-deprecated)
