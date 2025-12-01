MODULE_TOPDIR = ../grass
ORBBEC_SDK_PATH = /opt/OrbbecSDK_v2.4.11

ORBBEC_INCLUDE := -I$(ORBBEC_SDK_PATH)/include
ORBBEC_LIB_PATH := -L$(ORBBEC_SDK_PATH)/lib
ORBBEC_LIB := -lOrbbecSDK

PGM = r.in.kinect

LIBES = $(IMAGERYLIB) $(RASTERLIB) $(INTERPFLLIB) $(QTREELIB) $(QTREEDEP) $(GMATHLIB) $(INTERPDATALIB) $(VECTORLIB) $(DBMILIB) $(GISLIB) $(OMPLIB) $(ORBBEC_LIB_PATH) $(ORBBEC_LIB) -rdynamic -lk4a -lpcl_common -Wl,-Bstatic -lflann_cpp_s -Wl,-Bdynamic -lpcl_io -lpcl_kdtree -lpcl_octree -lpcl_search -lpcl_surface -lpcl_sample_consensus  -lpcl_filters -lpcl_features -lpcl_keypoints  -lpcl_segmentation -lpcl_tracking -lpthread -lboost_system -Wl,-Bdynamic  -ldl -Wl,-rpath,/usr/local/lib -Wl,-rpath,$(ORBBEC_SDK_PATH)/lib
DEPENDENCIES = $(IMAGERYDEP) $(RASTERDEP) $(INTERPFLDEP) $(QTREEDEP) $(INTERPDATADEP) $(GMATHDEP) $(VECTORDEP) $(DBMIDEP) $(GISDEP)
EXTRA_INC = $(VECT_INC) -I/usr/local/include/pcl-1.15 -I/usr/include/eigen3 $(ORBBEC_INCLUDE)
EXTRA_CFLAGS = -Wall -g -std=c++20 -march=native -Wno-deprecated -O0 $(VECT_CFLAGS) $(OMPCFLAGS) -static-libasan

LDFLAGS += -L/usr/local/lib -L/usr/lib $(ORBBEC_LIB_PATH)
include $(MODULE_TOPDIR)/include/Make/Module.make

LINK = $(CXX)

ifneq ($(strip $(CXX)),)
default: cmd
endif