MODULE_TOPDIR = ../../..

PGM = r.in.kinect

LIBES = $(RASTERLIB) $(GISLIB) $(VECTORLIB) -lk4a
DEPENDENCIES = $(RASTERDEP) $(GISDEP) $(VECTORDEP)

EXTRA_INC = $(VECT_INC)
EXTRA_CFLAGS = $(VECT_CFLAGS)

include $(MODULE_TOPDIR)/include/Make/Module.make

default: cmd
