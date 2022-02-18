#ifndef BINNING_H
#define BINNING_H

#include <math.h>

extern "C" {
#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>
#include <grass/glocale.h>
}

void *get_cell_ptr(void *array, int cols, int row, int col,
                   RASTER_MAP_TYPE map_type)
{
    return G_incr_void_ptr(array,
                           ((row * (size_t) cols) +
                            col) * Rast_cell_size(map_type));
}

void compute_weights(int size, float power, double **weights_array)
{

    for (int i = 0; i < size * 2 + 1; i++) {
        for (int j = 0; j < size * 2 + 1; j++) {
            double dist = sqrt((i - size) * (i - size) + (j - size) * (j - size));
            weights_array[i][j] = 1 / pow(dist, power);
        }
    }
    weights_array[size][size] = 1;
}

void fill_idw(void *sum_array, void *n_array, void *interp_array,
              int rows, int cols, int window_size, int method, double **weights_matrix)
{
    compute_weights(window_size, 2, weights_matrix);
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            size_t offset = (row * cols + col) * Rast_cell_size(FCELL_TYPE);
            size_t n_offset = (row * cols + col) * Rast_cell_size(CELL_TYPE);
            int n = Rast_get_c_value(G_incr_void_ptr(n_array, n_offset), CELL_TYPE);
            double sum =
                Rast_get_d_value(G_incr_void_ptr(sum_array, offset), FCELL_TYPE);
            if (n == 0) {
                int count = 0;
                double sum2 = 0;
                int nn;
                double weights_sum = 0;
                int w_row = 0;
                for (int rr = row - window_size; rr <= row + window_size; rr++, w_row++) {
                    int w_col = 0;
                    for (int cc = col - window_size; cc <= col + window_size; cc++, w_col++) {
                        if (cc < 0 || rr < 0 || cc >= cols || rr >= rows)
                            continue;
                        void *ptr_n = get_cell_ptr(n_array, cols, rr, cc, CELL_TYPE);
                        void *ptr_sum = get_cell_ptr(sum_array, cols, rr, cc, FCELL_TYPE);
                        if ((nn = Rast_get_c_value(ptr_n, CELL_TYPE))) {
                            if (method == 0)
                                sum2 += (Rast_get_f_value(ptr_sum, FCELL_TYPE) / nn) * weights_matrix[w_row][w_col];
                            else
                                sum2 += (Rast_get_f_value(ptr_sum, FCELL_TYPE)) * weights_matrix[w_row][w_col];
                            weights_sum += weights_matrix[w_row][w_col];
                            count += 1;
                        }
                    }
                }
                if (count > 2)
                    Rast_set_d_value(G_incr_void_ptr(interp_array, offset), sum2 / weights_sum, FCELL_TYPE);
                else
                    Rast_set_null_value(G_incr_void_ptr(interp_array, offset), 1, FCELL_TYPE);
            }
        }
    }
}

template<typename PointT>
inline void binning(pcl::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                    char* output, struct bound_box *bbox, double resolution,
                    double scale, double zexag, double bottom, double offset, const char *method_name,
                    bool interpolate, double **weights_matrix) {

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

    int method = 0; // mean
    if (strcmp(method_name, "min") == 0)
        method = 1; // min
    else if (strcmp(method_name, "max") == 0)
        method = 2; // max

    /* open output map */
    int out_fd = Rast_open_new(output, FCELL_TYPE);

    /* allocate memory for a single row of output data */
    void *raster_row = Rast_allocate_output_buf(FCELL_TYPE);

    void *n_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                             Rast_cell_size(CELL_TYPE));
    void *sum_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                               Rast_cell_size(FCELL_TYPE));
    void *interp_array = G_calloc((size_t) cellhd.rows * (cellhd.cols + 1),
                                  Rast_cell_size(FCELL_TYPE));
    int arr_row, arr_col;
    double z;
    for (int i = 0; i < cloud->points.size(); i++) {
        /* find the bin in the current array box */
        arr_row = (int)((cellhd.north - cloud->points[i].y) / cellhd.ns_res);
        arr_col = (int)((cloud->points[i].x - cellhd.west) / cellhd.ew_res);

        if (arr_row < 0 || arr_row >= cellhd.rows || arr_col < 0 || arr_col >= cellhd.cols){
            continue;
        }
        z = (cloud->points[i].z - bottom) * scale / zexag + offset;

        void *ptr_n = get_cell_ptr(n_array, cellhd.cols, arr_row, arr_col, CELL_TYPE);
        CELL old_n = Rast_get_c_value(ptr_n, CELL_TYPE);
        Rast_set_c_value(ptr_n, (1 + old_n), CELL_TYPE);

        void *ptr_sum = get_cell_ptr(sum_array, cellhd.cols, arr_row, arr_col, FCELL_TYPE);
        FCELL old_sum = Rast_get_f_value(ptr_sum, FCELL_TYPE);
        if (method == 0 || old_n == 0)
            Rast_set_f_value(ptr_sum, (z + old_sum), FCELL_TYPE);
        else if (method == 1)
            Rast_set_f_value(ptr_sum, z < old_sum ? z : old_sum, FCELL_TYPE);
        else
            Rast_set_f_value(ptr_sum, z > old_sum ? z : old_sum, FCELL_TYPE);
    }
    if (interpolate) {
        /* fill small holes */
        fill_idw(sum_array, n_array, interp_array, cellhd.rows, cellhd.cols, 1, method, weights_matrix);

        /* fill holes of max size 3 cm */
        int fill_size = (int) (0.015 / resolution);
        if (fill_size)
            fill_idw(sum_array, n_array, interp_array, cellhd.rows, cellhd.cols, fill_size, method, weights_matrix);
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
                if (!interpolate || Rast_is_f_null_value(G_incr_void_ptr(interp_array, offset))) {
                    Rast_set_null_value(ptr, 1, FCELL_TYPE);
                }
                else
                    Rast_set_d_value(ptr, Rast_get_d_value(G_incr_void_ptr(interp_array, offset), FCELL_TYPE), FCELL_TYPE);
            }
            else {
                if (method == 0)
                    Rast_set_d_value(ptr, (sum / n), FCELL_TYPE);
                else
                    Rast_set_d_value(ptr, sum, FCELL_TYPE);
            }

            ptr = G_incr_void_ptr(ptr, Rast_cell_size(FCELL_TYPE));
        }
        /* write out line of raster data */
        Rast_put_row(out_fd, raster_row, FCELL_TYPE);
    }

    /* free memory */
    G_free(n_array);
    G_free(sum_array);
    G_free(interp_array);
    G_free(raster_row);
    /* close raster file & write history */
    Rast_close(out_fd);

    /* colortable for elevations
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
            Rast_add_f_color_rule(&data1, 50, 121, 70,
                                  &data2, 90, 148, 80, &colors);
            break;
        case 2:
            Rast_add_f_color_rule(&data1, 90, 148, 80,
                                  &data2, 148, 174, 92, &colors);
            break;
        case 3:
            Rast_add_f_color_rule(&data1, 148, 174, 92,
                                  &data2, 224, 205, 103, &colors);
            break;
        case 4:
            Rast_add_f_color_rule(&data1, 224, 205, 103,
                                  &data2, 186, 151, 74, &colors);
            break;
        case 5:
            Rast_add_f_color_rule(&data1, 186, 151, 74,
                                  &data2, 159, 100, 44, &colors);
            break;
        }
    }
    const char *mapset = G_find_file("cell", output, "");
    Rast_write_colors(output, mapset, &colors);
    Rast_quantize_fp_map_range(output, mapset,
                (DCELL) zmin - 0.5, (DCELL) zmax + 0.5,
                (CELL) (zmin - 0.5), (CELL) (zmax + 0.5));*/
}

#endif // BINNING_H
