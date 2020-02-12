#ifndef BINNING_H
#define BINNING_H

#include <k4a/k4a.h>

#include <grass/raster.h>
#include <grass/vector.h>


void *get_cell_ptr(void *array, int cols, int row, int col,
                   RASTER_MAP_TYPE map_type);

void binning(k4a_float3_t *cloud, unsigned npoints,
             char* output, struct bound_box *bbox, double resolution,
             double scale, double zexag, double bottom, double offset,
             const char *method_name);

#endif // BINNING_H
