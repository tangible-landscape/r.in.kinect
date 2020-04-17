# r.in.kinect
GRASS GIS module for importing data from Microsoft Kinect v2 into a GRASS GIS raster or vector map.

## Dependencies
 * [libfreenect2](https://github.com/OpenKinect/libfreenect2)
 * [PCL](http://pointclouds.org)
 * OpenCV
 * [GRASS GIS 7](https://grass.osgeo.org)

## Installation
Supported platforms include Ubuntu Linux and Mac OS X. Running r.in.kinect on any other platforms is possible but not tested.

For Ubuntu 18.04, you can use [install shell script](https://raw.githubusercontent.com/tangible-landscape/tangible-landscape-install/master/install_Ubuntu-18.04_xbox-one.sh)
 to install all dependencies and r.in.kinect. It will also install [GRASS GIS Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape).

Otherwise, please follow official installation guides of dependencies. First test libfreenect2's Protonect binary before you proceed with other steps. Module r.in.kinect can then be downloaded and compiled:

    git clone https://github.com/tangible-landscape/r.in.kinect.git
    cd r.in.kinect
    make MODULE_TOPDIR=../path/to/grass
    make install MODULE_TOPDIR=../path/to/grass

You might need to edit the Makefile when an error ocurrs, check specifically you are using the installed PCL version.

