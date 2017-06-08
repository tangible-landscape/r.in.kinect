# r.in.kinect
GRASS GIS module for importing data from Microsoft Kinect v2 into a GRASS GIS raster or vector map.

## Dependencies
 * [libfreenect2](https://github.com/OpenKinect/libfreenect2)
 * [PCL](http://pointclouds.org)
 * OpenCV
 * [GRASS GIS 7](https://grass.osgeo.org)

## Installation
Supported platforms include Ubuntu Linux and Mac OS X. Running r.in.kinect on any other platforms is possible but not tested.

For Ubuntu 16.04, you can use [install.sh script](https://raw.githubusercontent.com/tangible-landscape/grass-tangible-landscape/master/install.sh)
 to install all dependencies and r.in.kinect.

Otherwise, please follow official installation guides of dependencies. First test libfreenect2's Protonect binary before you proceed with other steps. Here are notes specific to Ubuntu and Mac OS X.

### Ubuntu 16.04
Installation was tested on older Ubuntu 15.10 and 14.04, but 16.04 is easier to setup because new versions of libraries are already there.
#### libfreenect2
Follow [installation notes for libfreenect2](https://github.com/OpenKinect/libfreenect2#debianubuntu-1404), here is a short version:

Install dependencies:

    sudo apt-get install build-essential cmake pkg-config git libusb-1.0-0-dev libturbojpeg libjpeg-turbo8-dev libglfw3-dev

Download libfreenect2 source code (latest release) from here:
https://github.com/OpenKinect/libfreenect2/releases
and unpack it. Go to that directory and run:

    mkdir build && cd build
    cmake .. 
    make
    sudo make install
    
Set up udev rules for device access:

    sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
then replug the Kinect. Run the test program: 

    ./bin/Protonect


<!--Also I experienced problem with nvidia drivers. I used proprietary drivers installed through "Additional drivers" dialog but then I needed to uninstall package nvidia-opencl-idc-? and install ocl-icd-opencl-dev (but not sure if that did the trick)-->

#### PCL
Once libfreenect2 Protonect binary is working, install PCL. It is recomended to compile it instead using PPA. Tested is PCL 1.7.2. and 1.8.0 It needs some dependencies:

    sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev
  
Check other PCL dependencies [here](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php) and install whatever you might need for different PCL components.
Download PCL latest release from https://github.com/PointCloudLibrary/pcl/releases. Go to extracted folder and run:

    cd pcl-pcl-1.8.0 && mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j4
    sudo make -j2 install
#### opencv
We also need opencv:

    sudo apt-get install libopencv-dev

#### GRASS GIS
Then install GRASS GIS 7.2. First install [dependencies](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#Current_stable_Ubuntu_version) including [PROJ4, GEOS, GDAL](https://grasswiki.osgeo.org/wiki/Compile_and_Install_Ubuntu#Using_pre-compiled_dev_Packages_for_PROJ.4.2C_GEOS_and_GDAL). 

    sudo apt-get install \
       build-essential \
       flex make bison gcc libgcc1 g++ cmake ccache \
       python python-dev \
       python-opengl \
       python-wxversion python-wxtools python-wxgtk3.0 \
       python-dateutil libgsl-dev python-numpy \
       wx3.0-headers wx-common libwxgtk3.0-dev \
       libwxbase3.0-dev   \
       libncurses5-dev \
       zlib1g-dev gettext \
       libtiff5-dev libpnglite-dev \
       libcairo2 libcairo2-dev \
       sqlite3 libsqlite3-dev \
       libpq-dev \
       libreadline6 libreadline6-dev libfreetype6-dev \
       libfftw3-3 libfftw3-dev \
       libboost-thread-dev libboost-program-options-dev liblas-c-dev \
       resolvconf \
       libjasper-dev \
       subversion \
       libav-tools libavutil-dev ffmpeg2theora \
       libffmpegthumbnailer-dev \
       libavcodec-dev \
       libxmu-dev \
       libavformat-dev libswscale-dev \
       checkinstall \
       libglu1-mesa-dev libxmu-dev \
       ghostscript \
       libproj-dev proj-data proj-bin \
       libgeos-dev \
       libgdal-dev python-gdal gdal-bin

Then download GRASS GIS with subversion, configure and compile:

    svn checkout https://svn.osgeo.org/grass/grass/branches/releasebranch_7_2 grass72_release
    cd grass72_release
    CFLAGS="-O2 -Wall" LDFLAGS="-s" ./configure \
      --enable-largefile=yes \
      --with-nls \
      --with-cxx \
      --with-readline \
      --with-pthread \
      --with-proj-share=/usr/share/proj \
      --with-geos=/usr/bin/geos-config \
      --with-wxwidgets \
      --with-cairo \
      --with-opengl-libs=/usr/include/GL \
      --with-freetype=yes --with-freetype-includes="/usr/include/freetype2/" \
      --with-postgres=yes --with-postgres-includes="/usr/include/postgresql" \
      --with-sqlite=yes \
      --with-mysql=yes --with-mysql-includes="/usr/include/mysql" \
      --with-odbc=no \
      --with-liblas=yes --with-liblas-config=/usr/bin/liblas-config
    make -j4
    sudo make install
    
#### r.in.kinect
Finally clone this repository:

    git clone https://github.com/tangible-landscape/r.in.kinect.git
    cd r.in.kinect
    make MODULE_TOPDIR=../path/to/grass
    make install MODULE_TOPDIR=../path/to/grass
    
You might need to edit the Makefile when an error ocurrs, check specifically for PCL version.


### Mac OSX
Most libraries can be installed using homebrew, for libfreenect2 follow their  [guide](https://github.com/OpenKinect/libfreenect2#mac-osx). You might need to unplug Kinect and plug it [again](https://github.com/OpenKinect/libfreenect2#protonect-complains-about-no-device-connected-or-failure-opening-device) to make it work. Install PCL with package manager (I used Homebrew). GRASS GIS can be either compiled from source, or use [homebrew](https://grasswiki.osgeo.org/wiki/Compiling_on_MacOSX_using_homebrew).

Then clone this repository, cd into it and switch Makefile:

    mv Makefile.macosx Makefile

Run make, include path to GRASS, in case of GRASS installed with homebrew:

    make MODULE_TOPDIR=/usr/local/Cellar/grass-71/HEAD/grass-7.1.svn/
    
Then start GRASS GIS with `grass71` and r.in.kinect should be working.




