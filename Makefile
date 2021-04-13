MODULE_TOPDIR = ../..

PGM = r.in.kinect

LIBES = $(IMAGERYLIB) $(RASTERLIB) $(INTERPFLLIB) $(QTREELIB) $(QTREEDEP) $(GMATHLIB) $(INTERPDATALIB) $(VECTORLIB) $(DBMILIB) $(GISLIB) $(OMPLIB) -rdynamic  /usr/local/lib/libfreenect2.so  -lpcl_common -Wl,-Bstatic -lflann_cpp_s -Wl,-Bdynamic -lpcl_io -lpcl_kdtree -lpcl_octree -lpcl_search -lpcl_surface -lpcl_sample_consensus  -lpcl_filters -lpcl_features -lpcl_keypoints  -lpcl_segmentation -lpcl_tracking -lboost_system -lpthread  -Wl,-Bdynamic   /usr/lib/x86_64-linux-gnu/libopencv_core.so -ldl -Wl,-rpath,/usr/local/lib
DEPENDENCIES = $(IMAGERYDEP) $(RASTERDEP) $(INTERPFLDEP) $(QTREEDEP) $(INTERPDATADEP) $(GMATHDEP) $(VECTORDEP) $(DBMIDEP) $(GISDEP)
EXTRA_INC = $(VECT_INC) -I/usr/include/opencv4 -I/usr/local/include/pcl-1.11 -I/usr/include/eigen3
EXTRA_CFLAGS = -std=c++14 -march=native  -Wno-deprecated -O0 $(VECT_CFLAGS) $(OMPCFLAGS)

include $(MODULE_TOPDIR)/include/Make/Module.make

LINK = $(CXX)

ifneq ($(strip $(CXX)),)
default: cmd
endif
