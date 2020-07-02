# r.in.kinect
GRASS GIS module for importing data from Azure Kinect DK into a GRASS GIS raster or vector map.

## Dependencies
 * [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)
 * [PCL 1.10 and higher](http://pointclouds.org)
 * [GRASS GIS 7.8 and higher](https://grass.osgeo.org)

## Installation
Supported platforms include Ubuntu Linux. Running r.in.kinect on any other platforms is possible but not tested.

For Ubuntu 18.04, you can use [install shell script](https://raw.githubusercontent.com/tangible-landscape/tangible-landscape-install/master/install_Ubuntu-18.04_k4a.sh)
 to install all dependencies and r.in.kinect. It will also install [GRASS GIS Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape).

Otherwise, please follow official installation guides of dependencies. Module r.in.kinect can then be downloaded and compiled:

    git clone --single-branch --branch pcl-k4a https://github.com/tangible-landscape/r.in.kinect.git
    cd r.in.kinect
    make MODULE_TOPDIR=../path/to/grass
    make install MODULE_TOPDIR=../path/to/grass
    
You might need to edit the Makefile when an error ocurrs, check specifically you are using the installed PCL version.





