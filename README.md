# r.in.kinect
GRASS tool for importing data from a depth camera into a GRASS raster or vector map.

This branch (`femto-bolt`) supports the [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/) depth camera.
Other branches support the Microsoft Azure Kinect DK (`k4a`) and the Kinect for Xbox One (`xbox-one`).

See the [r.in.kinect manual page](r.in.kinect.md) for usage, options, and the calibration workflow.
It is the scanning backend of the [Tangible Landscape](https://tangible-landscape.github.io/) project.

## Dependencies
 * [Orbbec SDK v2](https://github.com/orbbec/OrbbecSDK_v2)
 * [PCL 1.15 and higher](http://pointclouds.org)
 * [GRASS 8.4 and higher](https://grass.osgeo.org)

## Installation
Supported platforms include Ubuntu Linux. Running r.in.kinect on any other platforms is possible but not tested.

For Ubuntu 24.04, you can use the [install shell script](https://github.com/tangible-landscape/tangible-landscape-install/blob/master/install_Ubuntu-24.04_femto-bolt.sh)
to install all dependencies and r.in.kinect. It will also install the [GRASS Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape).

Otherwise, install the dependencies following their official installation guides. Then download and compile the tool:

    git clone --single-branch --branch femto-bolt https://github.com/tangible-landscape/r.in.kinect.git
    cd r.in.kinect
    make MODULE_TOPDIR=../path/to/grass ORBBEC_SDK_PATH=/path/to/OrbbecSDK
    make install MODULE_TOPDIR=../path/to/grass ORBBEC_SDK_PATH=/path/to/OrbbecSDK

`ORBBEC_SDK_PATH` must point to the Orbbec SDK installation; it defaults to
`/opt/OrbbecSDK_v2.4.11` in the Makefile. You might need to edit the Makefile when an
error occurs, check specifically that you are using the installed PCL version.

To use r.in.kinect interactively for tangible modeling, also install the
[GRASS Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape).
