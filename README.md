# r.in.kinect
GRASS tool for importing data from a depth camera into a GRASS raster or vector map.

This branch (`femto-bolt`) supports the [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/) depth camera.
Other branches support the Microsoft Azure Kinect DK (`k4a`) and the Kinect for Xbox One (`xbox-one`).

See the [r.in.kinect manual page](r.in.kinect.md) for usage, options, and the calibration workflow.
It is the scanning backend of the [Tangible Landscape](https://tangible-landscape.github.io/) project.

## Dependencies
 * [Orbbec SDK v2](https://github.com/orbbec/OrbbecSDK_v2)
 * [PCL 1.15 and higher](http://pointclouds.org)
 * [GRASS 8.5 and higher](https://grass.osgeo.org) (cmake-based build)

## Installation
Supported platforms include Ubuntu Linux. Running r.in.kinect on any other platforms is possible but not tested.

For Ubuntu 24.04, you can use the [install shell script](https://github.com/tangible-landscape/tangible-landscape-install/blob/master/install_Ubuntu-24.04_femto-bolt.sh)
to install all dependencies and r.in.kinect. It will also install the [GRASS Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape).

Otherwise, install the dependencies following their official installation guides, then build and install r.in.kinect with `g.extension`:

    git clone --single-branch --branch femto-bolt https://github.com/tangible-landscape/r.in.kinect.git
    sudo OrbbecSDK_DIR=/opt/OrbbecSDK_v2.8.6/lib \
        grass --tmp-project XY --exec \
        g.extension -s extension=r.in.kinect url=$(pwd)/r.in.kinect

`OrbbecSDK_DIR` must point to the directory containing `OrbbecSDKConfig.cmake`
in your Orbbec SDK installation (adjust the version to match). PCL and GRASS
libraries are located automatically through their cmake config files. The `-s`
flag installs system-wide (under `$GISBASE`); drop it to install for the current
user only (and remove `sudo`).

To use r.in.kinect interactively for tangible modeling, also install the
[GRASS Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape).
