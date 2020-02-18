MODULE_TOPDIR = ../..

PGM = r.in.kinect

LIBES = $(IMAGERYLIB) $(RASTERLIB) $(INTERPFLLIB) $(QTREELIB) $(QTREEDEP) $(GMATHLIB) $(INTERPDATALIB) $(VECTORLIB) $(DBMILIB) $(GISLIB) $(OMPLIB) -rdynamic -lk4a -lpcl_common -Wl,-Bstatic -lflann_cpp_s -Wl,-Bdynamic -lpcl_io -lpcl_kdtree -lpcl_octree -lpcl_search -lpcl_surface -lpcl_sample_consensus  -lpcl_filters -lpcl_features -lpcl_keypoints  -lpcl_segmentation -lpcl_tracking -lboost_system -lboost_thread -lboost_date_time -lboost_iostreams -lboost_chrono -lpthread  -Wl,-Bdynamic  -ldl -Wl,-rpath,/usr/local/lib
DEPENDENCIES = $(IMAGERYDEP) $(RASTERDEP) $(INTERPFLDEP) $(QTREEDEP) $(INTERPDATADEP) $(GMATHDEP) $(VECTORDEP) $(DBMIDEP) $(GISDEP)
EXTRA_INC = $(VECT_INC) -I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/local/include -I/usr/include/libdrm
EXTRA_CFLAGS = -std=c++11 -march=native  -Wno-deprecated -O0 $(VECT_CFLAGS) $(OMPCFLAGS)

include $(MODULE_TOPDIR)/include/Make/Module.make

LINK = $(CXX)

ifneq ($(strip $(CXX)),)
default: cmd
endif
