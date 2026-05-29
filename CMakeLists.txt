cmake_minimum_required(VERSION 3.22)

project(r.in.kinect LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(build_addon)

find_M()
find_OpenMP()

find_package(PCL 1.15 REQUIRED COMPONENTS
  common
  io
  octree
  filters
  surface
  segmentation
  sample_consensus
  kdtree
  search)

find_package(Eigen3 REQUIRED)

find_package(OrbbecSDK REQUIRED)

add_library(PCL_ALL INTERFACE IMPORTED)
set_target_properties(PCL_ALL PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES      "${PCL_LIBRARIES}"
  INTERFACE_COMPILE_DEFINITIONS "${PCL_DEFINITIONS}")

set(sources
  main.cpp
  drawing.cpp
  interp.cpp
  analyses.h
  binning.h
  binning_color.h
  calibrate.h
  color_octree.h
  drawing.h
  femto-driver.h
  interp.h)

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
  DEPENDS
    PCL_ALL
    Eigen3::Eigen
    ob::OrbbecSDK
    LIBM
  OPTIONAL_DEPENDS
    OpenMP::OpenMP_CXX
  DOCFILES
    ${PROJECT_NAME}.html
    ${PROJECT_NAME}.md)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wno-deprecated)
