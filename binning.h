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

    struct Cell_head cellhd;

    G_get_set_window(&cellhd);
    cellhd.north = bbox->N;
    cellhd.south = bbox->S;
    cellhd.west = bbox->W;
    cellhd.east = bbox->E;
    cellhd.ns_res = resolution;
    cellhd.ew_res = resolution;
    G_adjust_Cell_head(&cellhd, 0, 0);
    Rast_set_window(&cellhd);
    /* open output map */
    int out_fd = Rast_open_new(output, FCELL_TYPE);

    /* allocate memory for a single row of output data */
    void *raster_row = Rast_allocate_output_buf(FCELL_TYPE);

    void *n_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                             Rast_cell_size(CELL_TYPE));
    void *sum_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                               Rast_cell_size(FCELL_TYPE));

    int arr_row, arr_col;
    double z;
    for (int i = 0; i < cloud.points.size(); i++) {
        /* find the bin in the current array box */
        arr_row = (int)((cellhd.north - cloud.points[i].y) / cellhd.ns_res);
        arr_col = (int)((cloud.points[i].x - cellhd.west) / cellhd.ew_res);

        if (arr_row < 0 || arr_row >= cellhd.rows || arr_col < 0 || arr_col >= cellhd.cols){
            continue;
        }
        z = (cloud.points[i].z - bbox->B) * scale / zexag + offset;

        void *ptr_n = get_cell_ptr(n_array, cellhd.cols, arr_row, arr_col, CELL_TYPE);
        CELL old_n = Rast_get_c_value(ptr_n, CELL_TYPE);
        Rast_set_c_value(ptr_n, (1 + old_n), CELL_TYPE);

        void *ptr_sum = get_cell_ptr(sum_array, cellhd.cols, arr_row, arr_col, FCELL_TYPE);
        FCELL old_sum = Rast_get_f_value(ptr_sum, FCELL_TYPE);
        Rast_set_f_value(ptr_sum, (z + old_sum), FCELL_TYPE);
    }

    /* calc stats and output */
    G_message(_("Writing to map ..."));
    for (int row = 0; row < cellhd.rows; row++) {
        void *ptr = raster_row;
        for (int col = 0; col < cellhd.cols; col++) {
            size_t offset = (row * cellhd.cols + col) * Rast_cell_size(FCELL_TYPE);
            size_t n_offset = (row * cellhd.cols + col) * Rast_cell_size(CELL_TYPE);
            int n = Rast_get_c_value(G_incr_void_ptr(n_array, n_offset), CELL_TYPE);
            double sum =
                Rast_get_d_value(G_incr_void_ptr(sum_array, offset), FCELL_TYPE);

            if (n == 0) {
                int count = 0;
                double sum2 = 0;
                int window_size = 1;
                int nn;
                for (int rr = row - window_size; rr <= row + window_size; rr++) {
                    for (int cc = col - window_size; cc <= col + window_size; cc++) {
                        if (cc < 0 || rr < 0 || cc >= cellhd.cols || rr >= cellhd.rows)
                            continue;
                        void *ptr2 = get_cell_ptr(n_array, cellhd.cols,
                                            rr, cc, CELL_TYPE);
                        void *ptr3 = get_cell_ptr(sum_array, cellhd.cols,
                                            rr, cc, FCELL_TYPE);
                        if ((nn = Rast_get_c_value(ptr2, CELL_TYPE))) {
                            sum2 += (Rast_get_f_value(ptr3, FCELL_TYPE) / nn);
                            count += 1;
                        }
                    }
                }
                if (count >= 3) {
                    Rast_set_f_value(ptr, sum2 / count, FCELL_TYPE);
                }
                else
                    Rast_set_null_value(ptr, 1, FCELL_TYPE);
            }
            else
                Rast_set_d_value(ptr, (sum / n), FCELL_TYPE);

            ptr = G_incr_void_ptr(ptr, Rast_cell_size(FCELL_TYPE));
        }
        /* write out line of raster data */
        Rast_put_row(out_fd, raster_row, FCELL_TYPE);
    }

    /* free memory */
    G_free(n_array);
    G_free(sum_array);
    G_free(raster_row);
    /* close raster file & write history */
    Rast_close(out_fd);

    /* colortable for elevations */
    struct Colors colors;
    struct FPRange range;
    double zmin, zmax;
    Rast_init_colors(&colors);
    Rast_read_fp_range(output, "", &range);
    Rast_get_fp_range_min_max(&range, &zmin, &zmax);

    double zstep = (FCELL) (zmax - zmin) / 5.;
    for (int j = 1; j <= 5; j++) {
        FCELL data1 = (FCELL) (zmin + (j - 1) * zstep);
        FCELL data2 = (FCELL) (zmin + j * zstep);
        switch (j) {
        case 1:
            Rast_add_f_color_rule(&data1, 0, 191, 191,
                                  &data2, 0, 255, 0, &colors);
            break;
        case 2:
            Rast_add_f_color_rule(&data1, 0, 255, 0,
                                  &data2, 255, 255, 0, &colors);
            break;
        case 3:
            Rast_add_f_color_rule(&data1, 255, 255, 0,
                                  &data2, 255, 127, 0, &colors);
            break;
        case 4:
            Rast_add_f_color_rule(&data1, 255, 127, 0,
                                  &data2, 191, 127, 63, &colors);
            break;
        case 5:
            Rast_add_f_color_rule(&data1, 191, 127, 63,
                                  &data2, 200, 200, 200, &colors);
            break;
        }
    }
    const char *mapset = G_find_file("cell", output, "");
    Rast_write_colors(output, mapset, &colors);
    Rast_quantize_fp_map_range(output, mapset,
                (DCELL) zmin - 0.5, (DCELL) zmax + 0.5,
                (CELL) (zmin - 0.5), (CELL) (zmax + 0.5));
}

#endif // BINNING_H
