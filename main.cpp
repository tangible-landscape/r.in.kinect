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


#include "k2g.h"

extern "C" {
#include <grass/gis.h>
#include <grass/vector.h>
#include <grass/glocale.h>
#include "interp.h"
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

template<typename PointT>
inline void trimNSEW(boost::shared_ptr<pcl::PointCloud<PointT>> cloud, double trim_N, double trim_S, double  trim_E, double trim_W) {

    double minX, maxX, minY, maxY, minZ, maxZ;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_pass (new pcl::PointCloud<PointT>);
    getMinMax(*cloud, minX, maxX, minY, maxY, minZ, maxZ);
    pcl::PassThrough<PointT> pass;
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

template<typename PointT>
inline void rotate_Z(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double angle) {

    Eigen::Affine3f transform_Z = Eigen::Affine3f::Identity();
    // The same rotation matrix as before; tetha radians arround Z axis
    transform_Z.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ()));
    transform_Z.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX()));

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_Z);
    transformed_cloud.swap (cloud);
}

template<typename PointT>
inline void trim_Z(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double zrange_min, double zrange_max) {

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_pass (new pcl::PointCloud<PointT>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zrange_min, zrange_max);
    pass.filter (*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);
}

template<typename PointT>
inline void smooth(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double radius) {

    // Create a KD-Tree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    boost::shared_ptr<pcl::PointCloud<PointT>> mls_points (new pcl::PointCloud<PointT>);

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
    struct Option *voutput_opt, *zrange_opt, *trim_opt, *rotate_Z_opt,
            *smooth_radius_opt;
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

    smooth_radius_opt = G_define_option();
    smooth_radius_opt->key = "smooth_radius";
    smooth_radius_opt->type = TYPE_DOUBLE;
    smooth_radius_opt->required = NO;
    smooth_radius_opt->label = _("Smooth radius");
    smooth_radius_opt->description = _("Recommended values between 0.006-0.009");

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
    double angle = pcl::deg2rad(atof(rotate_Z_opt->answer));

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
        trim_Z(cloud, zrange_min, zrange_max);
    }

    // rotation Z
    if (rotate_Z_opt->answer != NULL) {
        rotate_Z(cloud, angle);
    }

    // trim edges
    if (trim_opt->answer != NULL) {
        trimNSEW(cloud, trim_N, trim_S, trim_W, trim_E);
    }

    if (smooth_radius_opt->answer)
        smooth(cloud, atof(smooth_radius_opt->answer));

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
    interpolate(&Map, voutput_opt->answer, 20, 2, 60, 40, -1);

    k2g.shutDown();
    Vect_build(&Map);
    Vect_close(&Map);
    return EXIT_SUCCESS;
}
