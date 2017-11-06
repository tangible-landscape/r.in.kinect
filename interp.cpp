#include <unistd.h>
#if defined(_OPENMP)
#include <omp.h>
#endif
extern "C" {
#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>
#include <grass/glocale.h>
#include <grass/interpf.h>
#include <grass/gmath.h>
#include <grass/qtree.h>
#include <grass/dataquad.h>
}

int deallocate(struct multtree *tree)
{
    int j;

    if (tree == NULL)
        return 0;
    if (tree->data == NULL)
        return 0;
    if (tree->leafs != NULL) {
        for (j = 0; j < 4; j++) {
            deallocate(tree->leafs[j]);
        }
        G_free(tree->leafs);
    }
    else {
        G_free(tree->data->points);
        G_free(tree->data);

    }
    G_free(tree);
    return 1;
}

void interpolate(struct Map_info *Map, char* output, double tension,
                 double smoothing, int npmin, int segmax, double dmin,
                 struct bound_box *bbox, double resolution, int threads){

    struct interp_params params;
    struct Cell_head cellhd;

    G_get_set_window(&cellhd);
    cellhd.north = bbox->N;
    cellhd.south = bbox->S;
    cellhd.west = bbox->W;
    cellhd.east = bbox->E;
    cellhd.ns_res = resolution;
    cellhd.ew_res = resolution;
    G_adjust_Cell_head(&cellhd, 0, 0);

    double ew_res = cellhd.ew_res;
    double ns_res = cellhd.ns_res;
    int n_cols = cellhd.cols;
    int n_rows = cellhd.rows;
    double x_orig = cellhd.west;
    double y_orig = cellhd.south;
    double xm = cellhd.east;
    double ym = cellhd.north;
    double dmax;
    int npoint = 0;
    if (dmin <= 0) {
        if (ew_res < ns_res)
            dmin = ew_res / 2;
        else
            dmin = ns_res / 2;
    }
    dmax = dmin * 5;
    dmin = dmin * dmin;

#if defined(_OPENMP)
    omp_set_num_threads(threads);
#endif
    double *az = NULL;

    az = G_alloc_vector(n_cols + 1);
    if (!az) {
        G_fatal_error(_("Not enough memory for %s"), "az");
    }
    char *tmp = G_tempfile();
    FILE *Tmp_fd_z = fopen(tmp, "w+");
    if (!Tmp_fd_z)
        G_fatal_error(_("Unable to open temporary file <%s>"), tmp);

    struct quaddata *data;
    struct multfunc *functions;
    struct multtree *tree;
    struct multtree *root;
    struct tree_info *info;
    if ((data =	quad_data_new(x_orig, y_orig, xm, ym, n_rows, n_cols, 0, segmax)) == NULL)
        G_fatal_error(_("Unable to create quaddata"));
    if ((functions = MT_functions_new(quad_compare, quad_divide_data, quad_add_data,
                                      quad_intersect, quad_division_check,
                                      quad_get_points)) == NULL)
        G_fatal_error(_("Unable to create quadfunc"));

    if ((tree = MT_tree_new(data, NULL, NULL, 0)) == NULL)
        G_fatal_error(_("Unable to create tree"));
    root = tree;

    if ((info = MT_tree_info_new(root, functions, dmin, segmax)) == NULL)
        G_fatal_error(_("Unable to create tree info"));

    // initialization
    params.zmult = 1;
    params.kmin = npmin;
    params.kmax = segmax;
    params.nsizr = n_rows;
    params.nsizc = n_cols;
    params.az = az;
    params.fi = tension;
    params.KMAX2 = 2 * npmin;
    params.scik1 = 1;
    params.scik2 = 1;
    params.scik3 = 1;
    params.rsm = smoothing;
    params.elev = output;
    params.dmin = dmin;
    params.x_orig = x_orig;
    params.y_orig = y_orig;
    params.deriv = 0;
    params.theta = 0;
    params.scalex = 1;
    params.Tmp_fd_z = Tmp_fd_z;
    params.cv = 0;
    params.adx = NULL;
    params.ady = NULL;
    params.adxx = NULL;
    params.adyy = NULL;
    params.adxy = NULL;
    params.fdinp = NULL;
    params.elatt = 1;
    params.smatt = 1;
    params.maskmap = NULL;
    params.Tmp_fd_dx = NULL;
    params.Tmp_fd_dy = NULL;
    params.Tmp_fd_xx = NULL;
    params.Tmp_fd_yy = NULL;
    params.Tmp_fd_xy = NULL;
    params.fddevi = NULL;
    params.ts = NULL;
    params.slope = NULL;
    params.aspect = NULL;
    params.pcurv = NULL;
    params.tcurv = NULL;
    params.mcurv = NULL;
    params.grid_calc = IL_grid_calc_2d;
    params.matrix_create = IL_matrix_create;
    params.check_points = IL_check_at_points_2d;
    params.secpar = IL_secpar_loop_2d;
    params.interp = IL_crst;
    params.interpder = IL_crstg;
    params.wr_temp = IL_write_temp_2d;

    // segmentation
    double xmin, xmax, ymin, ymax, zmin, zmax;
    int totsegm = IL_vector_input_data_2d(&params, Map, 0,
                                          NULL, NULL,
                                          info, &xmin, &xmax,
                                          &ymin, &ymax, &zmin, &zmax, &npoint, &dmax);
    double ertot, zminac, zmaxac;
    double gmin, gmax, c1min, c1max, c2min, c2max;
    double deltx, delty;
    deltx = xmax - xmin;
    delty = ymax - ymin;
    double dnorm =  sqrt((deltx * delty * npmin) / npoint);
#if defined(_OPENMP)
    IL_interp_segments_2d_parallel(&params, info, info->root, NULL,
                          zmin, zmax, &zminac, &zmaxac, &gmin, &gmax,
                          &c1min, &c1max, &c2min, &c2max, &ertot, totsegm,
                          n_cols, dnorm, threads);
#else
    IL_interp_segments_2d(&params, info, info->root, NULL,
                          zmin, zmax, &zminac, &zmaxac, &gmin, &gmax,
                          &c1min, &c1max, &c2min, &c2max, &ertot, totsegm,
                          n_cols, dnorm);
#endif
    G_free_vector(az);
    IL_output_2d(&params, &cellhd, zmin, zmax, zminac, zmaxac, c1min,
                 c1max, c2min, c2max, gmin, gmax, ertot, output, dnorm,
                 0, 1, npoint);
    fclose(Tmp_fd_z);
    unlink(tmp);
    deallocate(tree);

}
