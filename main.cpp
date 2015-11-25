/*
 ****************************************************************************
 *
 * MODULE:       v.in.kinect
 * AUTHOR(S):    Anna Petrasova
 * PURPOSE:      Import points as vector from Kinect v2
 * COPYRIGHT:    (C) 2015 by the GRASS Development Team
 *
 *               This program is free software under the GNU General
 *               Public License (>=v2). Read the file COPYING that
 *               comes with GRASS for details.
 *
 *****************************************************************************/


/* using the most-specific-first order of includes */
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>


#include "k2g.h"

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

#include <stdlib.h>



template <typename PointT> inline void
getMinMax(const pcl::PointCloud< PointT > &cloud, double &minX, double &maxX,
          double &minY, double &maxY, double &minZ, double &maxZ) {
    PointT minp, maxp;
    pcl::getMinMax3D (cloud, minp, maxp);
    minX = minp.x;
    minY = minp.y;
    minZ = minp.z;
    maxX = maxp.x;
    maxY = maxp.y;
    maxZ = maxp.z;
}

int main(int argc, char **argv)
{
    struct GModule *module;
    struct Option *voutput_opt, *zrange_opt, *trim_opt, *rotate_Z_opt;
    struct Map_info Map;
    struct line_pnts *Points;
    struct line_cats *Cats;
    int cat;

    G_gisinit(argv[0]);

    module = G_define_module();
    G_add_keyword(_("vector"));
    G_add_keyword(_("scan"));
    G_add_keyword(_("points"));
    module->label = _("Imports a point cloud from Kinect v2");
    module->description = _("Imports a point cloud from Kinect v2");

    zrange_opt = G_define_option();
    zrange_opt->key = "zrange";
    zrange_opt->type = TYPE_DOUBLE;
    zrange_opt->required = NO;
    zrange_opt->key_desc = "min,max";
    zrange_opt->label = _("Filter range for z data (min,max)");
    zrange_opt->description = _("Z is distance from scanner in meters");

    trim_opt = G_define_option();
    trim_opt->key = "trim";
    trim_opt->type = TYPE_DOUBLE;
    trim_opt->required = NO;
    trim_opt->key_desc = "N,S,E,W";
    trim_opt->description = _("Trim edges in mm");

    rotate_Z_opt = G_define_option();
    rotate_Z_opt->key = "rotate";
    rotate_Z_opt->type = TYPE_DOUBLE;
    rotate_Z_opt->required = NO;
    rotate_Z_opt->description = _("Rotate along Z axis");

    voutput_opt = G_define_standard_option(G_OPT_V_OUTPUT);
    if (G_parser(argc, argv))
        exit(EXIT_FAILURE);

    if (Vect_open_new(&Map, voutput_opt->answer, WITH_Z) < 0)
        G_fatal_error(_("Unable to create vector map <%s>"), voutput_opt->answer);
    Vect_hist_command(&Map);

    Points = Vect_new_line_struct();
    Cats = Vect_new_cats_struct();

    /* parse zrange */
    double zrange_min, zrange_max;
    if (zrange_opt->answer != NULL) {
        zrange_min = atof(zrange_opt->answers[0]);
        zrange_max = atof(zrange_opt->answers[1]);
    }

    /* parse trim */
    double trim_N, trim_S, trim_E, trim_W;
    if (trim_opt->answer != NULL) {
        trim_N = atof(trim_opt->answers[0])/1000;
        trim_S = atof(trim_opt->answers[1])/1000;
        trim_E = atof(trim_opt->answers[2])/1000;
        trim_W = atof(trim_opt->answers[3])/1000;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pass (new pcl::PointCloud<pcl::PointXYZRGB>);


    K2G k2g(OPENGL);
    cloud = k2g.getCloud();
    cloud->sensor_orientation_.w() = 0.0;
    cloud->sensor_orientation_.x() = 1.0;
    cloud->sensor_orientation_.y() = 0.0;
    cloud->sensor_orientation_.z() = -1.0;

    // remove invalid points
    std::vector<int> index_nans;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index_nans);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    sor.filter(*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);

    // trim Z
    if (zrange_opt->answer != NULL) {
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(zrange_min, zrange_max);
        pass.filter (*cloud_filtered_pass);
        cloud_filtered_pass.swap (cloud);
    }

    // rotation Z
    if (rotate_Z_opt->answer != NULL) {
        double angle = pcl::deg2rad(atof(rotate_Z_opt->answer));
        Eigen::Affine3f transform_Z = Eigen::Affine3f::Identity();
        // The same rotation matrix as before; tetha radians arround Z axis
        transform_Z.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ()));
        transform_Z.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX()));

        // Executing the transformation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud (*cloud, *transformed_cloud, transform_Z);
        transformed_cloud.swap (cloud);
    }
    // trim edges
    if (trim_opt->answer != NULL) {
        double minX, maxX, minY, maxY, minZ, maxZ;
        getMinMax(*cloud, minX, maxX, minY, maxY, minZ, maxZ);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(minX + trim_W, maxX - trim_E);
        pass.filter (*cloud_filtered_pass);
        cloud_filtered_pass.swap (cloud);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(minY + trim_S, maxY - trim_N);
        pass.filter (*cloud_filtered_pass);
        cloud_filtered_pass.swap (cloud);
    }

    for (int i; i < cloud->points.size(); i++) {
        Vect_reset_line(Points);
        Vect_reset_cats(Cats);

        Vect_append_point(Points, cloud->points[i].x,
                          cloud->points[i].y,
                          cloud->points[i].z);
        Vect_cat_set(Cats, 1, cat);
        Vect_write_line(&Map, GV_POINT, Points, Cats);
    }
    Vect_rewind(&Map);
    // interpolation
    struct interp_params params;
    static struct Cell_head cellhd;
    G_get_set_window(&cellhd);

    double ew_res = cellhd.ew_res;
    double ns_res = cellhd.ns_res;
    int n_cols = cellhd.cols;
    int n_rows = cellhd.rows;
    double x_orig = cellhd.west;
    double y_orig = cellhd.south;
    double xm = cellhd.east;
    double ym = cellhd.north;
    double dmin, dmax;
    int npoint = 0;
    if (ew_res < ns_res)
        dmin = ew_res / 2;
    else
        dmin = ns_res / 2;
    dmax = dmin * 5;
    double *az = NULL, *adx = NULL, *ady = NULL, *adxx = NULL, *adyy = NULL,
        *adxy = NULL;
    double tension = 30;
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

    if ((data =	quad_data_new(x_orig, y_orig, xm, ym, n_rows, n_cols, 0, 40)) == NULL)
        G_fatal_error(_("Unable to create quaddata"));
    if ((functions = MT_functions_new(quad_compare, quad_divide_data, quad_add_data,
                                      quad_intersect, quad_division_check,
                                      quad_get_points)) == NULL)
        G_fatal_error(_("Unable to create quadfunc"));

    if ((tree = MT_tree_new(data, NULL, NULL, 0)) == NULL)
        G_fatal_error(_("Unable to create tree"));
    root = tree;

    if ((info = MT_tree_info_new(root, functions, dmin, 40)) == NULL)
        G_fatal_error(_("Unable to create tree info"));

    double xmin, xmax, ymin, ymax, zmin, zmax;
    IL_init_params_2d(&params, NULL, 1, 1, 1, 80, 40, NULL, n_rows,
                      n_cols, az, adx, ady, adxx, adyy, adxy, tension, 160,
                      1, 1, 1, 1, voutput_opt->answer, NULL, NULL, NULL,
                      NULL, NULL, dmin, x_orig, y_orig, 0, 0,
                      1, Tmp_fd_z, NULL, NULL, NULL,
                      NULL, NULL, NULL, NULL, 0, "");
    IL_init_func_2d(&params, IL_grid_calc_2d, IL_matrix_create,
                    IL_check_at_points_2d, IL_secpar_loop_2d, IL_crst,
                    IL_crstg, IL_write_temp_2d);
    int totsegm = IL_vector_input_data_2d(&params, &Map, 0,
                                          NULL, NULL,
                                          info, &xmin, &xmax,
                                          &ymin, &ymax, &zmin, &zmax, &npoint, &dmax);
    double ertot, zminac, zmaxac;
    double gmin, gmax, c1min, c1max, c2min, c2max;
    double deltx, delty;
    deltx = xmax - xmin;
    delty = ymax - ymin;
    double dnorm =  sqrt((deltx * delty * 80) / npoint);
    std::cout <<totsegm<< std::endl;
    std::cout << x_orig<< std::endl;
    std::cout << y_orig<< std::endl;
    std::cout << npoint<< std::endl;

    IL_interp_segments_2d(&params, info, info->root, NULL,
                          zmin, zmax, &zminac, &zmaxac, &gmin, &gmax,
                          &c1min, &c1max, &c2min, &c2max, &ertot, totsegm,
                          n_cols, dnorm);
    G_free_vector(az);
    int ii = IL_output_2d(&params, &cellhd, zmin, zmax, zminac, zmaxac, c1min,
                          c1max, c2min, c2max, gmin, gmax, ertot, voutput_opt->answer, dnorm,
                          0, 1, npoint);
    fclose(Tmp_fd_z);
    unlink(tmp);
    k2g.shutDown();
    Vect_build(&Map);
    Vect_close(&Map);
    return EXIT_SUCCESS;
}
