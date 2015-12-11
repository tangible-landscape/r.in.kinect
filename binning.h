#ifndef BINNING_H
#define BINNING_H

extern "C" {
#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>
#include <grass/glocale.h>
#include "binning_support.h"
}

template<typename PointT>
inline void binning(pcl::PointCloud< PointT > &cloud,
                    char* output, struct bound_box *bbox, double resolution,
                    double scale, double zexag, double offset) {

    struct PointBinning point_binning;
    struct Cell_head cellhd;

    G_get_set_window(&cellhd);
    cellhd.north = bbox->N;
    cellhd.south = bbox->S;
    cellhd.west = bbox->W;
    cellhd.east = bbox->E;
    cellhd.ns_res = resolution;
    cellhd.ew_res = resolution;
    G_adjust_Cell_head(&cellhd, 0, 0);
    G_set_window(&cellhd);

    point_binning_set(&point_binning, "mean", nullptr, nullptr);

    /* open output map */
    int out_fd = Rast_open_new(output, FCELL_TYPE);

    /* allocate memory for a single row of output data */
    void *raster_row = Rast_allocate_output_buf(FCELL_TYPE);

    point_binning_allocate(&point_binning, cellhd.rows, cellhd.cols, FCELL_TYPE);

    int arr_row, arr_col;
    double z;
    for (int i = 0; i < cloud.points.size(); i++) {
        /* find the bin in the current array box */
        arr_row = (int)((cellhd.north - cloud.points[i].y) / cellhd.ns_res);
        arr_col = (int)((cloud.points[i].x - cellhd.west) / cellhd.ew_res);

        z = (cloud.points[i].z - bbox->B) * scale / zexag + offset;

        update_value(&point_binning, nullptr, cellhd.cols, arr_row, arr_col,
                     FCELL_TYPE, z);
    }
    /* calc stats and output */
    G_message(_("Writing to map ..."));
    for (int row = 0; row < cellhd.rows; row++) {
        write_values(&point_binning, nullptr, raster_row, row, cellhd.cols, FCELL_TYPE);
        /* write out line of raster data */
        Rast_put_row(out_fd, raster_row, FCELL_TYPE);
    }

    /* free memory */
    point_binning_free(&point_binning, nullptr);
    G_free(raster_row);

    /* close raster file & write history */
    Rast_close(out_fd);
}

#endif // BINNING_H
