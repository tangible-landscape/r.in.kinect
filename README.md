# v.in.kinect
GRASS GIS module for importing data from Microsoft Kinect v2 into a GRASS GIS raster or vector map.

## Dependencies
 * [libfreenect2](https://github.com/OpenKinect/libfreenect2)
 * [PCL](pointclouds.org)
 * OpenCV
 * [GRASS GIS 7](grass.osgeo.org)

## Installation
Supported platforms include Ubuntu Linux and Mac OS X. Running r.in.kinect on any other platforms is possible but not tested.

Please follow official installation guides of dependencies. First test libfreenect2's Protonect binary before you proceed with other steps. Here are notes specific to Ubuntu and Mac OS X.

### Ubuntu
Installation was tested on Ubuntu 15.10 and 14.04. Newer versions might be easier to setup because often newer versions of libraries are recommended. It might be necessary to add several PPAs and if they don't have the right PPA for your Ubuntu versions, you can change it in [synaptic](http://askubuntu.com/a/293210). Follow [installation notes for libfreenect2](https://github.com/OpenKinect/libfreenect2#debianubuntu-1404) and it might be neccessary to change /etc/udev/rules.d as [described](https://github.com/OpenKinect/libfreenect2#protonect-complains-about-no-device-connected-or-failure-opening-device).

Once libfreenect2 Protonect binary is working, install PCL. It is recomended to compile it instead using PPA. Tested is PCL 1.7.2. It needs some dependencies:

    sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev
  
Check other PCL dependencies [here](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php) and install whatever you might need for different PCL components.

Then install GRASS GIS 7. First install [dependencies](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#Current_stable_Ubuntu_version) including [PROJ4, GEOS, GDAL](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#Using_pre-compiled_dev_Packages_for_PROJ.4.2C_GEOS_and_GDAL). Then configure GRASS GIS, use [command here](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#GRASS_GIS) but scroll down to GRASS GIS 7 example configuration and do what the note below says. Then run `make`.

Finally clone this repository and try to compile it with `make MODULE_TOPDIR=path/to/grass`. You might need to edit the Makefile when an error ocurrs. Finally, `make install` in GRASS folder is optional, without it you can launch GRASS as `./bin.x86_64-pc-linux-gnu/grass71.





