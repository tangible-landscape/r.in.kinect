# r.in.kinect
GRASS GIS module for importing data from Microsoft Kinect v2 into a GRASS GIS raster or vector map.

## Dependencies
 * [libfreenect2](https://github.com/OpenKinect/libfreenect2)
 * [PCL](http://pointclouds.org)
 * OpenCV
 * [GRASS GIS 7](https://grass.osgeo.org)

## Installation
Supported platforms include Ubuntu Linux and Mac OS X. Running r.in.kinect on any other platforms is possible but not tested.

Please follow official installation guides of dependencies. First test libfreenect2's Protonect binary before you proceed with other steps. Here are notes specific to Ubuntu and Mac OS X.

### Ubuntu
Installation was tested on Ubuntu 15.10 and 14.04. Newer versions might be easier to setup because often newer versions of libraries are recommended. It might be necessary to add several PPAs and if they don't have the right PPA for your Ubuntu versions, you can change it in [synaptic](http://askubuntu.com/a/293210). Follow [installation notes for libfreenect2](https://github.com/OpenKinect/libfreenect2#debianubuntu-1404) and it might be neccessary to change /etc/udev/rules.d as [described](https://github.com/OpenKinect/libfreenect2#protonect-complains-about-no-device-connected-or-failure-opening-device). Also I experienced problem with nvidia drivers. I used proprietary drivers installed through "Additional drivers" dialog but then I needed to uninstall package nvidia-opencl-idc-? and install ocl-icd-opencl-dev (but not sure if that did the trick)

Once libfreenect2 Protonect binary is working, install PCL. It is recomended to compile it instead using PPA. Tested is PCL 1.7.2. It needs some dependencies:

    sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev
  
Check other PCL dependencies [here](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php) and install whatever you might need for different PCL components.

We also need opencv:

    sudo apt-get install libopencv-dev

Then install GRASS GIS 7.2. First install [dependencies](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#Current_stable_Ubuntu_version) including [PROJ4, GEOS, GDAL](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#Using_pre-compiled_dev_Packages_for_PROJ.4.2C_GEOS_and_GDAL). Then download GRASS GIS with subversion:

    svn checkout https://svn.osgeo.org/grass/grass/branches/releasebranch_7_2 grass72_release
    
configure GRASS GIS, use [command here](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#GRASS_GIS) but scroll down to GRASS GIS 7 example configuration (not GRASS 6) and follow the note below the configure command. Then run `make`.

Finally clone this repository and try to compile it with `make MODULE_TOPDIR=path/to/grass`. You might need to edit the Makefile when an error ocurrs. Finally, `make install` in GRASS folder is optional, without it you can launch GRASS as `./bin.x86_64-pc-linux-gnu/grass71.


### Mac OSX
Most libraries can be installed using homebrew, for libfreenect2 follow their  [guide](https://github.com/OpenKinect/libfreenect2#mac-osx). You might need to unplug Kinect and plug it [again](https://github.com/OpenKinect/libfreenect2#protonect-complains-about-no-device-connected-or-failure-opening-device) to make it work. Install PCL with package manager (I used Homebrew). GRASS GIS can be either compiled from source, or use [homebrew](https://grasswiki.osgeo.org/wiki/Compiling_on_MacOSX_using_homebrew).

Then clone this repository, cd into it and switch Makefile:

    mv Makefile.macosx Makefile

Run make, include path to GRASS, in case of GRASS installed with homebrew:

    make MODULE_TOPDIR=/usr/local/Cellar/grass-71/HEAD/grass-7.1.svn/
    
Then start GRASS GIS with `grass71` and r.in.kinect should be working.




