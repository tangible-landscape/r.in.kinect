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
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/sac_segmentation.h>


#include "k2g.h"
#include "binning.h"
#include "calibrate.h"

extern "C" {
#include <grass/gis.h>
#include <grass/vector.h>
#include <grass/raster.h>
#include <grass/glocale.h>
#include "interp.h"
}

#include <stdlib.h>
#include <signal.h>

static volatile sig_atomic_t signaled = 0;
void terminate (int param)
{
    G_message(_("Scanning terminated"));
    signaled = 1;
}

template <typename PointT> inline void
getMinMax(const pcl::PointCloud< PointT > &cloud, struct bound_box &bbox) {
    PointT minp, maxp;
    pcl::getMinMax3D (cloud, minp, maxp);
    bbox.W = minp.x;
    bbox.S = minp.y;
    bbox.B = minp.z;
    bbox.E = maxp.x;
    bbox.N = maxp.y;
    bbox.T = maxp.z;
}

template<typename PointT>
inline void trimNSEW(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double trim_N, double trim_S, double  trim_E, double trim_W) {

    struct bound_box bbox;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_pass (new pcl::PointCloud<PointT>(512, 424));
    getMinMax(*cloud, bbox);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(bbox.W + trim_W, bbox.E - trim_E);
    pass.filter (*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(bbox.S + trim_S, bbox.N - trim_N);
    pass.filter (*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);
}

template<typename PointT>
inline void rotate_Z(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double angle) {

    Eigen::Affine3f transform_Z = Eigen::Affine3f::Identity();
    // The same rotation matrix as before; tetha radians arround Z axis
    transform_Z.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ()));

    // Executing the transformation
    typename pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>(512, 424));
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_Z);
    transformed_cloud.swap (cloud);
}

template<typename PointT>
inline void trim_Z(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double zrange_min, double zrange_max) {
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_pass (new pcl::PointCloud<PointT>(512, 424));
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-zrange_max, -zrange_min);
    pass.filter (*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);

}

template<typename PointT>
inline void smooth(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double radius) {

    // Create a KD-Tree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    boost::shared_ptr<pcl::PointCloud<PointT>> mls_points (new pcl::PointCloud<PointT>(512, 424));

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, PointT> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (false);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (radius);

    // Reconstruct
    mls.process (*mls_points);
    mls_points.swap(cloud);
}


int main(int argc, char **argv)
{
    struct GModule *module;
    struct Option *voutput_opt, *routput_opt, *zrange_opt, *trim_opt, *rotate_Z_opt,
            *smooth_radius_opt, *region_opt, *raster_opt, *zexag_opt, *resolution_opt,
            *method_opt, *calib_matrix_opt, *numscan_opt;
    struct Flag *loop_flag, *calib_flag;
    struct Map_info Map;
    struct line_pnts *Points;
    struct line_cats *Cats;
    int cat = 1;

    G_gisinit(argv[0]);

    module = G_define_module();
    G_add_keyword(_("vector"));
    G_add_keyword(_("scan"));
    G_add_keyword(_("points"));
    module->label = _("Imports a point cloud from Kinect v2");
    module->description = _("Imports a point cloud from Kinect v2");

    voutput_opt = G_define_standard_option(G_OPT_V_OUTPUT);
    voutput_opt->required = NO;
    voutput_opt->key = "vector";

    routput_opt = G_define_standard_option(G_OPT_R_OUTPUT);
    zrange_opt = G_define_option();
    zrange_opt->key = "zrange";
    zrange_opt->type = TYPE_DOUBLE;
    zrange_opt->required = NO;
    zrange_opt->key_desc = "min,max";
    zrange_opt->label = _("Filter range for z data (min,max)");
    zrange_opt->description = _("Z is distance from scanner in cm");

    trim_opt = G_define_option();
    trim_opt->key = "trim";
    trim_opt->type = TYPE_DOUBLE;
    trim_opt->required = NO;
    trim_opt->key_desc = "N,S,E,W";
    trim_opt->description = _("Trim edges in cm");

    rotate_Z_opt = G_define_option();
    rotate_Z_opt->key = "rotate";
    rotate_Z_opt->type = TYPE_DOUBLE;
    rotate_Z_opt->required = NO;
    rotate_Z_opt->answer = "0";
    rotate_Z_opt->description = _("Rotate along Z axis");

    smooth_radius_opt = G_define_option();
    smooth_radius_opt->key = "smooth_radius";
    smooth_radius_opt->type = TYPE_DOUBLE;
    smooth_radius_opt->required = NO;
    smooth_radius_opt->label = _("Smooth radius");
    smooth_radius_opt->description = _("Recommended values between 0.006-0.009");

    region_opt = G_define_option();
    region_opt->key = "region";
    region_opt->key_desc = "name";
    region_opt->required = NO;
    region_opt->multiple = NO;
    region_opt->type = TYPE_STRING;
    region_opt->description = _("Region of the resulting raster");
    region_opt->gisprompt = "old,windows,region";

    raster_opt = G_define_standard_option(G_OPT_R_MAP);
    raster_opt->key = "raster";
    raster_opt->required = NO;
    raster_opt->multiple = NO;
    raster_opt->description = _("Match resulting raster to this raster map");

    resolution_opt = G_define_option();
    resolution_opt->key = "resolution";
    resolution_opt->type = TYPE_DOUBLE;
    resolution_opt->required = YES;
    resolution_opt->answer = "0.002";
    resolution_opt->label = _("Raster resolution");
    resolution_opt->description = _("Recommended values between 0.001-0.003");

    zexag_opt = G_define_option();
    zexag_opt->key = "zexag";
    zexag_opt->type = TYPE_DOUBLE;
    zexag_opt->required = NO;
    zexag_opt->required = NO;
    zexag_opt->answer = "1";
    zexag_opt->description = _("Vertical exaggeration");

    method_opt = G_define_option();
    method_opt->key = "method";
    method_opt->multiple = NO;
    method_opt->required = NO;
    method_opt->type = TYPE_STRING;
    method_opt->options = "interpolation,mean,min,max";
    method_opt->answer = "interpolation";
    method_opt->description = _("Surface reconstruction method");

    calib_matrix_opt = G_define_option();
    calib_matrix_opt->key = "calib_matrix";
    calib_matrix_opt->multiple = YES;
    calib_matrix_opt->type = TYPE_DOUBLE;
    calib_matrix_opt->required = NO;
    calib_matrix_opt->description = _("Calibration matrix");

    numscan_opt = G_define_option();
    numscan_opt->answer = "1";
    numscan_opt->key = "numscan";
    numscan_opt->type = TYPE_INTEGER;
    numscan_opt->description = _("Number of scans to intergrate");
    numscan_opt->required = NO;

    loop_flag = G_define_flag();
    loop_flag->key = 'l';
    loop_flag->description = _("Keep scanning in a loop");

    calib_flag = G_define_flag();
    calib_flag->key = 'c';
    calib_flag->description = _("Calibrate");

    if (G_parser(argc, argv))
        exit(EXIT_FAILURE);

    Points = Vect_new_line_struct();
    Cats = Vect_new_cats_struct();

    /* parse zrange */
    double zrange_min, zrange_max;
    if (zrange_opt->answer != NULL) {
        zrange_min = atof(zrange_opt->answers[0])/100;
        zrange_max = atof(zrange_opt->answers[1])/100;
    }

    /* parse trim */
    double trim_N, trim_S, trim_E, trim_W;
    if (trim_opt->answer != NULL) {
        trim_N = atof(trim_opt->answers[0])/100;
        trim_S = atof(trim_opt->answers[1])/100;
        trim_E = atof(trim_opt->answers[2])/100;
        trim_W = atof(trim_opt->answers[3])/100;
    }
    double angle = pcl::deg2rad(atof(rotate_Z_opt->answer));
    double zexag = atof(zexag_opt->answer);
    Eigen::Matrix4f transform_matrix;
    if (calib_matrix_opt->answer) {
        transform_matrix = read_matrix(calib_matrix_opt);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(512, 424));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pass (new pcl::PointCloud<pcl::PointXYZRGB>(512, 424));

    struct bound_box bbox;
    struct Cell_head cellhd, window;
    double offset, scale;
    char *name;

    if ((name = region_opt->answer)){	/* region= */
        G_get_element_window(&window, "windows", name, "");
        offset = window.bottom;
    }
    else if ((name = raster_opt->answer)) {
        struct FPRange range;
        double zmin, zmax;
        Rast_get_cellhd(name, "", &window);
        Rast_read_fp_range(name, "", &range);
        Rast_get_fp_range_min_max(&range, &zmin, &zmax);
        offset = zmin;
    }
    else { // current region
        G_get_set_window(&window);
        offset = 0;
    }


    K2G k2g(OPENGL);
    k2g.getCloud(cloud);
    cloud->sensor_orientation_.w() = 0.0;
    cloud->sensor_orientation_.x() = 1.0;
    cloud->sensor_orientation_.y() = 0.0;
    cloud->sensor_orientation_.z() = 0.0;
    int j = 0;
    // get terminating signals
    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    int max_points = 0;
    while (j < 1) {
        if (signaled == 1) {
            break;
        }

        cloud = k2g.getCloud();
        for (int s = 0; s < atoi(numscan_opt->answer) - 1; s++){
            *(cloud) += *(k2g.getCloud());

        // remove invalid points
        std::vector<int> index_nans;

        pcl::removeNaNFromPointCloud(*cloud, *cloud, index_nans);

        // calibration
        if(calib_flag->answer) {
            calibrate(cloud);
            j++;
            continue;
        }
        // rotation of the point cloud based on calibration
        if (calib_matrix_opt->answer) {
            rotate_with_matrix(cloud, transform_matrix);
        }
        // trim Z
        if (zrange_opt->answer != NULL) {
            trim_Z(cloud, zrange_min, zrange_max);
        }

        // rotation Z
        rotate_Z(cloud, angle);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(0.5);
        sor.filter(*cloud_filtered_pass);
        cloud_filtered_pass.swap (cloud);



        // trim edges
        if (trim_opt->answer != NULL) {
            trimNSEW(cloud, trim_N, trim_S, trim_W, trim_E);
        }


//        if (max_points < cloud->points.size()) {
//            max_points = cloud->points.size();
//        }
//        std::cout << (max_points - cloud->points.size()) / float(max_points) << std::endl;
//        if ((max_points - cloud->points.size()) / float(max_points) > 0.01)
//            continue;

        if (smooth_radius_opt->answer)
            smooth(cloud, atof(smooth_radius_opt->answer));

        // get Z scaling
        getMinMax(*cloud, bbox);
        scale = ((window.north - window.south) / (bbox.N - bbox.S) +
                 (window.east - window.west) / (bbox.E - bbox.W)) / 2;

        // write to vector
        if (voutput_opt->answer || strcmp(method_opt->answer, "interpolation") == 0) {
            double z;
            if (voutput_opt->answer) {
                if (Vect_open_new(&Map, voutput_opt->answer, WITH_Z) < 0)
                    G_fatal_error(_("Unable to create temporary vector map <%s>"), voutput_opt->answer);
            }
            else {
                if (Vect_open_tmp_new(&Map, routput_opt->answer, WITH_Z) < 0)
                    G_fatal_error(_("Unable to create temporary vector map <%s>"), routput_opt->answer);
            }
            for (int i=0; i < cloud->points.size(); i++) {
                Vect_reset_line(Points);
                Vect_reset_cats(Cats);
                z = (cloud->points[i].z - bbox.B) * scale / zexag + offset;
                Vect_append_point(Points, cloud->points[i].x,
                                  cloud->points[i].y,
                                  z);
                Vect_cat_set(Cats, 1, cat);
                Vect_write_line(&Map, GV_POINT, Points, Cats);
            }
            if (strcmp(method_opt->answer, "interpolation") == 0) {
                // interpolate
                Vect_rewind(&Map);
                interpolate(&Map, routput_opt->answer, 20, 2, 50, 40, -1,
                            &bbox, atof(resolution_opt->answer));
            }
            Vect_close(&Map);
        }
        if (strcmp(method_opt->answer, "interpolation") != 0) {
            binning(*cloud, routput_opt->answer, &bbox, atof(resolution_opt->answer),
                    scale, zexag, offset, method_opt->answer);
        }

        // georeference horizontally
        Rast_get_cellhd(routput_opt->answer, "", &cellhd);
        window.rows = cellhd.rows;
        window.cols = cellhd.cols;
        G_adjust_Cell_head(&window, 1, 1);
        cellhd.north = window.north;
        cellhd.south = window.south;
        cellhd.east = window.east;
        cellhd.west = window.west;
        Rast_put_cellhd(routput_opt->answer, &cellhd);


        if (!loop_flag->answer)
            j++;
    }

    k2g.shutDown();

    return EXIT_SUCCESS;
}
