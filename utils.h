#ifndef UTILS_H
#define UTILS_H


#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>

#include <k4a/k4a.h>

void update_input_region(char* raster, struct Cell_head* window, double* offset);

void get_min_max(const k4a_float3_t *cloud, unsigned npoints, struct bound_box *bbox);

#endif // UTILS_H
