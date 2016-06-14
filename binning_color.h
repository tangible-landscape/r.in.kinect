#ifndef BINNING_COLOR_H
#define BINNING_COLOR_H

extern "C" {
#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>
#include <grass/glocale.h>
}
#include "binning.h"


char *get_color_name(const char* basename, const char* color) {

    char output[100];
    strcpy(output, basename);
    strcat(output, "_");
    strcat(output, color);
    return G_store(output);
}

template<typename PointT>
inline void binning_color(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                    char* output, struct bound_box *bbox, double resolution) {

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
    char* output_r = get_color_name(output, "r");
    char* output_g = get_color_name(output, "g");
    char* output_b = get_color_name(output, "b");
    int outr_fd = Rast_open_new(output_r, CELL_TYPE);
    int outg_fd = Rast_open_new(output_g, CELL_TYPE);
    int outb_fd = Rast_open_new(output_b, CELL_TYPE);

    /* allocate memory for a single row of output data */
    void *raster_row_r = Rast_allocate_output_buf(CELL_TYPE);
    void *raster_row_g = Rast_allocate_output_buf(CELL_TYPE);
    void *raster_row_b = Rast_allocate_output_buf(CELL_TYPE);

    void *n_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                             Rast_cell_size(CELL_TYPE));
    void *sum_r_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                                Rast_cell_size(CELL_TYPE));
    void *sum_g_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                                 Rast_cell_size(CELL_TYPE));
    void *sum_b_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                                 Rast_cell_size(CELL_TYPE));

    int arr_row, arr_col;
    int r, g, b;
    for (int i = 0; i < cloud->points.size(); i++) {
        /* find the bin in the current array box */
        arr_row = (int)((cellhd.north - cloud->points[i].y) / cellhd.ns_res);
        arr_col = (int)((cloud->points[i].x - cellhd.west) / cellhd.ew_res);
        Eigen::Vector3i rgbv = cloud->points[i].getRGBVector3i();
        g = rgbv[1];
        r = rgbv[0];
        b = rgbv[2];
        if (arr_row < 0 || arr_row >= cellhd.rows || arr_col < 0 || arr_col >= cellhd.cols){
            continue;
        }

        void *ptr_n = get_cell_ptr(n_array, cellhd.cols, arr_row, arr_col, CELL_TYPE);
        CELL old_n = Rast_get_c_value(ptr_n, CELL_TYPE);
        Rast_set_c_value(ptr_n, (1 + old_n), CELL_TYPE);

        void *ptr_sum_r = get_cell_ptr(sum_r_array, cellhd.cols, arr_row, arr_col, CELL_TYPE);
        CELL old_sum_r = Rast_get_c_value(ptr_sum_r, CELL_TYPE);
        Rast_set_c_value(ptr_sum_r, (r + old_sum_r), CELL_TYPE);

        void *ptr_sum_g = get_cell_ptr(sum_g_array, cellhd.cols, arr_row, arr_col, CELL_TYPE);
        CELL old_sum_g = Rast_get_c_value(ptr_sum_g, CELL_TYPE);
        Rast_set_c_value(ptr_sum_g, (g + old_sum_g), CELL_TYPE);

        void *ptr_sum_b = get_cell_ptr(sum_b_array, cellhd.cols, arr_row, arr_col, CELL_TYPE);
        CELL old_sum_b = Rast_get_c_value(ptr_sum_b, CELL_TYPE);
        Rast_set_c_value(ptr_sum_b, (b + old_sum_b), CELL_TYPE);
    }

    /* calc stats and output */
    G_message(_("Writing colors to RGB maps ..."));
    for (int row = 0; row < cellhd.rows; row++) {
        void *ptr_r = raster_row_r;
        void *ptr_g = raster_row_g;
        void *ptr_b = raster_row_b;
        for (int col = 0; col < cellhd.cols; col++) {
            size_t offset = (row * cellhd.cols + col) * Rast_cell_size(CELL_TYPE);
            size_t n_offset = (row * cellhd.cols + col) * Rast_cell_size(CELL_TYPE);
            int n = Rast_get_c_value(G_incr_void_ptr(n_array, n_offset), CELL_TYPE);
            int sum_r = Rast_get_c_value(G_incr_void_ptr(sum_r_array, offset), CELL_TYPE);
            int sum_g = Rast_get_c_value(G_incr_void_ptr(sum_g_array, offset), CELL_TYPE);
            int sum_b = Rast_get_c_value(G_incr_void_ptr(sum_b_array, offset), CELL_TYPE);

            if (n == 0) {
                int count = 0;
                int sum2_r = 0, sum2_g = 0, sum2_b = 0;
                int window_size = 1;
                int nn;
                for (int rr = row - window_size; rr <= row + window_size; rr++) {
                    for (int cc = col - window_size; cc <= col + window_size; cc++) {
                        if (cc < 0 || rr < 0 || cc >= cellhd.cols || rr >= cellhd.rows)
                            continue;
                        void *ptr2 = get_cell_ptr(n_array, cellhd.cols,
                                                  rr, cc, CELL_TYPE);
                        void *ptr_r = get_cell_ptr(sum_r_array, cellhd.cols,
                                                   rr, cc, CELL_TYPE);
                        void *ptr_g = get_cell_ptr(sum_g_array, cellhd.cols,
                                                   rr, cc, CELL_TYPE);
                        void *ptr_b = get_cell_ptr(sum_b_array, cellhd.cols,
                                                   rr, cc, CELL_TYPE);
                        if ((nn = Rast_get_c_value(ptr2, CELL_TYPE))) {
                            sum2_r += Rast_get_c_value(ptr_r, CELL_TYPE) / nn;
                            sum2_g += Rast_get_c_value(ptr_g, CELL_TYPE) / nn;
                            sum2_b += Rast_get_c_value(ptr_b, CELL_TYPE) / nn;
                            count += 1;
                        }
                    }
                }
                if (count >= 3) {
                    Rast_set_c_value(ptr_r, sum2_r / count, CELL_TYPE);
                    Rast_set_c_value(ptr_g, sum2_g / count, CELL_TYPE);
                    Rast_set_c_value(ptr_b, sum2_b / count, CELL_TYPE);
                }
                else {
                    Rast_set_null_value(ptr_r, 1, CELL_TYPE);
                    Rast_set_null_value(ptr_g, 1, CELL_TYPE);
                    Rast_set_null_value(ptr_b, 1, CELL_TYPE);
                }
            }
            else {
                Rast_set_c_value(ptr_r, (sum_r / n), CELL_TYPE);
                Rast_set_c_value(ptr_g, (sum_g / n), CELL_TYPE);
                Rast_set_c_value(ptr_b, (sum_b / n), CELL_TYPE);
            }

            ptr_r = G_incr_void_ptr(ptr_r, Rast_cell_size(CELL_TYPE));
            ptr_g = G_incr_void_ptr(ptr_g, Rast_cell_size(CELL_TYPE));
            ptr_b = G_incr_void_ptr(ptr_b, Rast_cell_size(CELL_TYPE));
        }
        /* write out line of raster data */
        Rast_put_row(outr_fd, raster_row_r, CELL_TYPE);
        Rast_put_row(outg_fd, raster_row_g, CELL_TYPE);
        Rast_put_row(outb_fd, raster_row_b, CELL_TYPE);
    }

    /* free memory */
    G_free(sum_b_array);
    G_free(sum_g_array);
    G_free(sum_r_array);
    G_free(n_array);
    G_free(raster_row_g);
    G_free(raster_row_b);
    G_free(raster_row_r);

    /* close raster file & write history */
    Rast_close(outb_fd);
    Rast_close(outg_fd);
    Rast_close(outr_fd);

    /* colortable for elevations */
    struct Colors colors;
    struct Range range;
    int zmin, zmax;
    const char *mapset = G_find_file("cell", output_g, "");
    Rast_init_colors(&colors);
    Rast_read_range(output_r, "", &range);
    Rast_get_range_min_max(&range, &zmin, &zmax);
    Rast_make_colors(&colors, "grey", zmin, zmax);
    Rast_write_colors(output_r, mapset, &colors);

    Rast_read_range(output_g, "", &range);
    Rast_get_range_min_max(&range, &zmin, &zmax);
    Rast_make_colors(&colors, "grey", zmin, zmax);
    Rast_write_colors(output_g, mapset, &colors);

    Rast_read_range(output_b, "", &range);
    Rast_get_range_min_max(&range, &zmin, &zmax);
    Rast_make_colors(&colors, "grey", zmin, zmax);
    Rast_write_colors(output_b, mapset, &colors);

    G_free(output_r);
    G_free(output_g);
    G_free(output_b);
}

#endif // BINNING_COLOR_H
