
#include "utils.h"

#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>

#include <k4a/k4a.h>

#include <float.h>

void update_input_region(char* raster, struct Cell_head *window, double* offset) {
   if (raster) {
        struct FPRange range;
        double zmin, zmax;
        Rast_get_cellhd(raster, "", window);
        Rast_read_fp_range(raster, "", &range);
        Rast_get_fp_range_min_max(&range, &zmin, &zmax);
        *offset = zmin;
    }
    else { // current region
        G_get_set_window(window);
        offset = 0;
    }
}


void get_min_max(const k4a_float3_t *cloud, unsigned npoints, struct bound_box *bbox) {
    
    double minx = DBL_MAX;
    double miny = DBL_MAX;
    double minz = DBL_MAX;
    double maxx = -DBL_MAX;
    double maxy = -DBL_MAX;
    double maxz = -DBL_MAX;
    
    for (int i = 0; i < npoints; i++) {
        float x = cloud[i].xyz.x;
        float y = cloud[i].xyz.y;
        float z = cloud[i].xyz.z;
        minx = (x < minx) ? x : minx;
        miny = (y < miny) ? y : miny;
        minz = (z < minz) ? z : minz;
        maxx = (x > maxx) ? x : maxx;
        maxy = (y > maxy) ? y : maxy;
        maxz = (z > maxz) ? z : maxz;
    }
        
    bbox->W = minx;
    bbox->S = miny;
    bbox->B = minz;
    bbox->E = maxx;
    bbox->N = maxy;
    bbox->T = maxz;
}