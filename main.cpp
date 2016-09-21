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
#include <pcl/io/ply_io.h>

#include "k2g.h"
#include "binning.h"
#include "binning_color.h"
#include "calibrate.h"
#include "interp.h"
#include "analyses.h"
#include "drawing.h"

extern "C" {
#include <grass/gis.h>
#include <grass/vector.h>
#include <grass/raster.h>
#include <grass/glocale.h>
}


#include <stdlib.h>
#include <signal.h>


static volatile sig_atomic_t signaled = 0;
static volatile sig_atomic_t signal_new_input = 0;

void terminate (int param)
{
    signaled = 1;
}

void signal_read_new_input (int param)
{
    signal_new_input = 1;
}

void update_input_region(char* raster, char* region, struct Cell_head &window, double &offset, bool &region3D) {
    if (region){	/* region= */
        G_get_element_window(&window, "windows", region, "");
        offset = window.bottom;
        if (window.top != window.bottom)
            region3D = true;
    }
    else if (raster) {
        struct FPRange range;
        double zmin, zmax;
        Rast_get_cellhd(raster, "", &window);
        Rast_read_fp_range(raster, "", &range);
        Rast_get_fp_range_min_max(&range, &zmin, &zmax);
        offset = zmin;
    }
    else { // current region
        G_get_set_window(&window);
        offset = 0;
    }
}

void get_draw_type(char* draw_type_string, int &vect_type){
    vect_type = -1;
    if (strcmp(draw_type_string, "point") == 0)
        vect_type = GV_POINT;
    else if (strcmp(draw_type_string, "line") == 0)
        vect_type = GV_LINE;
    else if (strcmp(draw_type_string, "area") == 0)
        vect_type = GV_AREA;
}
// read, parse and set values of scanning variables when new input from stdin comes
void read_new_input(char* &routput, double &zrange_min, double &zrange_max,
                    double &clip_N, double &clip_S, double &clip_E, double &clip_W,
                    double &trim_tolerance, double &rotate, double &zexag, char* &method,
                    int &numscan, double &smooth, double &resolution, bool &use_equalized,
                    struct Cell_head &window, double &offset, bool &region3D,
                    char* &color_output, char* &voutput, char * &ply,
                    char* &contours_output, double &contours_step,
                    int &draw_type, int &draw_threshold, char* &draw_output) {
    char buf[200];
    char **tokens;
    char **tokens2;
    int line;
    // TODO: this leaks

    for (line = 1;; line++) {
        if (G_getl2(buf, 200, stdin)) {
            if(buf[0] == '\0')
                break;
            tokens = G_tokenize(buf, "=");
            if (strcmp(tokens[0], "resolution") == 0)
                resolution = atof(tokens[1]);
            else if (strcmp(tokens[0], "smooth_radius") == 0)
                smooth = atof(tokens[1]);
            else if (strcmp(tokens[0], "output") == 0) {
                if (tokens[1][0] == '\0')
                    routput = NULL;
                else
                    routput = G_store(tokens[1]);
            }
            else if (strcmp(tokens[0], "zrange") == 0) {
                tokens2 = G_tokenize(tokens[1], ",");
                zrange_min = atof(tokens2[0])/100;
                zrange_max = atof(tokens2[1])/100;
                G_free_tokens(tokens2);
            }
            else if (strcmp(tokens[0], "trim") == 0) {
                tokens2 = G_tokenize(tokens[1], ",");
                clip_N = atof(tokens2[0])/100;
                clip_S = atof(tokens2[1])/100;
                clip_E = atof(tokens2[2])/100;
                clip_W = atof(tokens2[3])/100;
                G_free_tokens(tokens2);
            }
            else if (strcmp(tokens[0], "trim_tolerance") == 0)
                trim_tolerance = atof(tokens[1]);
            else if (strcmp(tokens[0], "rotate") == 0)
                rotate = pcl::deg2rad(atof(tokens[1]) + 180);
            else if (strcmp(tokens[0], "zexag") == 0)
                zexag = atof(tokens[1]);
            else if (strcmp(tokens[0], "method") == 0)
                method = G_store(tokens[1]);
            else if (strcmp(tokens[0], "numscan") == 0)
                numscan = atoi(tokens[1]);
            else if (strcmp(tokens[0], "region") == 0)
                update_input_region(NULL, tokens[1], window, offset, region3D);
            else if (strcmp(tokens[0], "raster") == 0)
                update_input_region(tokens[1], NULL, window, offset, region3D);
            // export
            else if (strcmp(tokens[0], "color_output") == 0) {
                if (tokens[1][0] == '\0')
                    color_output = NULL;
                else
                    color_output = G_store(tokens[1]);
            }
            else if (strcmp(tokens[0], "vector") == 0) {
                if (tokens[1][0] == '\0')
                    voutput = NULL;
                else
                    voutput = G_store(tokens[1]);
            }
            else if (strcmp(tokens[0], "ply") == 0) {
                if (tokens[1][0] == '\0')
                    ply = NULL;
                else
                    ply = G_store(tokens[1]);
            }
            else if (strcmp(tokens[0], "flags") == 0) {
                if (G_strcasestr(tokens[1], "e"))
                    use_equalized = true;
                else
                    use_equalized = false;
            }
            // contours
            else if (strcmp(tokens[0], "contours") == 0) {
                if (tokens[1][0] == '\0')
                    contours_output = NULL;
                else
                    contours_output = G_store(tokens[1]);
            }
            else if (strcmp(tokens[0], "contours_step") == 0)
                contours_step = atof(tokens[1]);
            // drawing
            else if (strcmp(tokens[0], "draw") == 0)
                get_draw_type(tokens[1], draw_type);
            else if (strcmp(tokens[0], "draw_threshold") == 0)
                draw_threshold = atoi(tokens[1]);
            else if (strcmp(tokens[0], "draw_output") == 0) {
                if (tokens[1][0] == '\0')
                    draw_output = NULL;
                else
                    draw_output = G_store(tokens[1]);
            }
            G_free_tokens(tokens);
        }
        else
            break;
    }
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
inline void clipNSEW(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double clip_N, double clip_S, double  clip_E, double clip_W) {

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_pass (new pcl::PointCloud<PointT>(512, 424));
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-clip_W, clip_E);
    pass.filter (*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-clip_S, clip_N);
    pass.filter (*cloud_filtered_pass);
    cloud_filtered_pass.swap (cloud);
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

int median(std::vector<int> &v)
{
    std::vector<int> new_;
    new_.reserve(v.size());
    for (int i = 0; i < v.size(); i++) {
        if (v[i] != 0)
            new_.push_back(v[i]);
    }

    size_t n = new_.size() / 2;
    nth_element(new_.begin(), new_.begin()+n, new_.end());
    return new_[n];
}

template<typename PointT>
void autotrim(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, double &clip_N, double &clip_S, double &clip_E, double &clip_W, double tolerance) {
    struct bound_box bbox;
    getMinMax(*cloud, bbox);
    double resolution = 0.003;
    int length_x = (int)((bbox.E - bbox.W) / resolution + 0.5);
    int length_y = (int)((bbox.N - bbox.S) / resolution + 0.5);
    std::vector<int> x_array(length_x, 0);
    std::vector<int> y_array(length_y, 0);

    unsigned int idx;
    for(typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it!= cloud->end(); it++){
        idx = int((it->x - bbox.W) / resolution);
        if (idx < x_array.size())
            x_array[idx] += 1;

        idx = int((it->y - bbox.S) / resolution);
        if (idx < y_array.size())
            y_array[idx] += 1;
    }
    int median_x = median(x_array);
    int median_y = median(y_array);

    clip_N = 0;
    clip_S = 0;
    clip_E = 0;
    clip_W = 0;
    for (int i = 0; i < x_array.size(); i++) {
        if (x_array[i] < tolerance * median_x) {
            clip_W = (i + 1) * resolution;
        }
        else
            break;
    }
    for (int i = x_array.size() - 1; i >= 0; i--) {
        if (x_array[i] < tolerance * median_x) {
            clip_E = (x_array.size() - i) * resolution;
        }
        else
            break;
    }
    for (int i = 0; i < y_array.size(); i++) {
        if (y_array[i] < tolerance * median_y) {
            clip_S = (i + 1) * resolution;
        }
        else
            break;
    }
    for (int i = y_array.size() - 1; i >= 0; i--) {
        if (y_array[i] < tolerance * median_y) {
            clip_N = (y_array.size() - i) * resolution;
        }
        else
            break;
    }
    //std::cout << clip_N << " " << clip_S << " " << clip_E << " " << clip_W << std::endl;

}

int main(int argc, char **argv)
{
    struct GModule *module;
    struct Option *voutput_opt, *routput_opt, *color_output_opt, *ply_opt, *zrange_opt, *trim_opt, *rotate_Z_opt,
            *smooth_radius_opt, *region_opt, *raster_opt, *zexag_opt, *resolution_opt,
            *method_opt, *calib_matrix_opt, *numscan_opt, *trim_tolerance_opt,
            *contours_map, *contours_step_opt, *draw_opt, *draw_vector_opt, *draw_threshold_opt;
    struct Flag *loop_flag, *calib_flag, *equalize_flag;
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

    routput_opt = G_define_standard_option(G_OPT_R_OUTPUT);
    routput_opt->guisection = _("Output");
    routput_opt->required = NO;

    resolution_opt = G_define_option();
    resolution_opt->key = "resolution";
    resolution_opt->type = TYPE_DOUBLE;
    resolution_opt->required = NO;
    resolution_opt->answer = "0.002";
    resolution_opt->label = _("Raster resolution");
    resolution_opt->description = _("Recommended values between 0.001-0.003");
    resolution_opt->guisection = _("Output");

    color_output_opt = G_define_standard_option(G_OPT_R_BASENAME_OUTPUT);
    color_output_opt->key = "color_output";
    color_output_opt->description = _("Basename for color output");
    color_output_opt->guisection = _("Output");
    color_output_opt->required = NO;

    voutput_opt = G_define_standard_option(G_OPT_V_OUTPUT);
    voutput_opt->required = NO;
    voutput_opt->key = "vector";
    voutput_opt->guisection = _("Output");

    ply_opt = G_define_standard_option(G_OPT_F_OUTPUT);
    ply_opt->required = NO;
    ply_opt->key = "ply";
    ply_opt->description = _("Name of output binary PLY file");
    ply_opt->guisection = _("Output");

    zrange_opt = G_define_option();
    zrange_opt->key = "zrange";
    zrange_opt->type = TYPE_DOUBLE;
    zrange_opt->required = NO;
    zrange_opt->key_desc = "min,max";
    zrange_opt->label = _("Filter range for z data (min,max)");
    zrange_opt->description = _("Z is distance from scanner in cm");
    zrange_opt->guisection = _("Filter");

    trim_opt = G_define_option();
    trim_opt->key = "trim";
    trim_opt->type = TYPE_DOUBLE;
    trim_opt->required = NO;
    trim_opt->key_desc = "N,S,E,W";
    trim_opt->description = _("Clip box from center in cm");
    trim_opt->guisection = _("Filter");

    trim_tolerance_opt = G_define_option();
    trim_tolerance_opt->key = "trim_tolerance";
    trim_tolerance_opt->type = TYPE_DOUBLE;
    trim_tolerance_opt->required = NO;
    trim_tolerance_opt->description = _("Influences how much are model sides trimmed automatically, "
        " should be higher for rectangular models");
    trim_tolerance_opt->label = _("Trim tolerance between 0 and 1");
    trim_tolerance_opt->options = "0-1";
    trim_tolerance_opt->guisection = _("Filter");

    rotate_Z_opt = G_define_option();
    rotate_Z_opt->key = "rotate";
    rotate_Z_opt->type = TYPE_DOUBLE;
    rotate_Z_opt->required = NO;
    rotate_Z_opt->answer = "0";
    rotate_Z_opt->description = _("Rotate along Z axis");
    rotate_Z_opt->guisection = _("Georeferencing");

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
    region_opt->guisection = _("Georeferencing");

    raster_opt = G_define_standard_option(G_OPT_R_MAP);
    raster_opt->key = "raster";
    raster_opt->required = NO;
    raster_opt->multiple = NO;
    raster_opt->description = _("Match resulting raster to this raster map");
    raster_opt->guisection = _("Georeferencing");

    zexag_opt = G_define_option();
    zexag_opt->key = "zexag";
    zexag_opt->type = TYPE_DOUBLE;
    zexag_opt->required = NO;
    zexag_opt->required = NO;
    zexag_opt->answer = "1";
    zexag_opt->description = _("Vertical exaggeration");
    zexag_opt->guisection = _("Georeferencing");

    method_opt = G_define_option();
    method_opt->key = "method";
    method_opt->multiple = NO;
    method_opt->required = NO;
    method_opt->type = TYPE_STRING;
    method_opt->options = "interpolation,mean,min,max";
    method_opt->answer = "mean";
    method_opt->description = _("Surface reconstruction method");

    calib_matrix_opt = G_define_option();
    calib_matrix_opt->key = "calib_matrix";
    calib_matrix_opt->multiple = YES;
    calib_matrix_opt->type = TYPE_DOUBLE;
    calib_matrix_opt->required = NO;
    calib_matrix_opt->description = _("Calibration matrix");
    calib_matrix_opt->guisection = _("Calibration");

    numscan_opt = G_define_option();
    numscan_opt->answer = "1";
    numscan_opt->key = "numscan";
    numscan_opt->type = TYPE_INTEGER;
    numscan_opt->description = _("Number of scans to intergrate");
    numscan_opt->required = NO;

    contours_map = G_define_standard_option(G_OPT_V_MAP);
    contours_map->key = "contours";
    contours_map->required = NO;
    contours_map->description = _("Name of contour vector map");

    contours_step_opt = G_define_option();
    contours_step_opt->key = "contours_step";
    contours_step_opt->description = _("Increment between contour levels");
    contours_step_opt->type = TYPE_DOUBLE;
    contours_step_opt->required = NO;

    equalize_flag = G_define_flag();
    equalize_flag->key = 'e';
    equalize_flag->description = _("Histogram equalized color table");

    loop_flag = G_define_flag();
    loop_flag->key = 'l';
    loop_flag->description = _("Keep scanning in a loop");

    calib_flag = G_define_flag();
    calib_flag->key = 'c';
    calib_flag->description = _("Calibrate");
    calib_flag->guisection = _("Calibration");

    draw_opt = G_define_option();
    draw_opt->key = "draw";
    draw_opt->description = _("Draw with laser pointer");
    draw_opt->type = TYPE_STRING;
    draw_opt->required = NO;
    draw_opt->options = "point,line,area";
    draw_opt->answer = "point";
    draw_opt->guisection = _("Drawing");

    draw_threshold_opt = G_define_option();
    draw_threshold_opt->key = "draw_threshold";
    draw_threshold_opt->description = _("Brightness threshold for detecting laser pointer");
    draw_threshold_opt->type = TYPE_INTEGER;
    draw_threshold_opt->required = YES;
    draw_threshold_opt->answer = "760";
    draw_threshold_opt->guisection = _("Drawing");

    draw_vector_opt = G_define_standard_option(G_OPT_V_OUTPUT);
    draw_vector_opt->key = "draw_output";
    draw_vector_opt->guisection = _("Drawing");
    draw_vector_opt->required = NO;

    G_option_required(calib_flag, routput_opt, voutput_opt, ply_opt, draw_vector_opt, NULL);
    G_option_requires(routput_opt, resolution_opt, NULL);
    G_option_requires(color_output_opt, resolution_opt, NULL);
    G_option_requires(contours_map, contours_step_opt, routput_opt, NULL);
    G_option_requires(equalize_flag, routput_opt, NULL);

    if (G_parser(argc, argv))
        exit(EXIT_FAILURE);

    // initailization of variables
    double resolution = 0.002;
    if (resolution_opt->answer)
        resolution = atof(resolution_opt->answer);
    double smooth_radius = 0.008;
    if (smooth_radius_opt->answer)
        smooth_radius = atof(smooth_radius_opt->answer);
    char* routput = NULL;
    if (routput_opt->answer)
        routput = routput_opt->answer;

    /* parse zrange */
    double zrange_min, zrange_max;
    if (zrange_opt->answer != NULL) {
        zrange_min = atof(zrange_opt->answers[0])/100;
        zrange_max = atof(zrange_opt->answers[1])/100;
    }

    /* parse trim */
    double clip_N, clip_S, clip_E, clip_W;
    if (trim_opt->answer != NULL) {
        clip_N = atof(trim_opt->answers[0])/100;
        clip_S = atof(trim_opt->answers[1])/100;
        clip_E = atof(trim_opt->answers[2])/100;
        clip_W = atof(trim_opt->answers[3])/100;
    }
    double trim_tolerance;
    if (trim_tolerance_opt->answer)
        trim_tolerance = atof(trim_tolerance_opt->answer);

    double angle = pcl::deg2rad(atof(rotate_Z_opt->answer) + 180);
    double zexag = atof(zexag_opt->answer);
    Eigen::Matrix4f transform_matrix;
    if (calib_matrix_opt->answer) {
        transform_matrix = read_matrix(calib_matrix_opt);
    }
    char *method = method_opt->answer;
    int numscan = atoi(numscan_opt->answer);
    char *color_output = color_output_opt->answer;
    char *voutput = voutput_opt->answer;
    char *ply = ply_opt->answer;
    char *contours_output = contours_map->answer;
    double contours_step;
    if (contours_output)
        contours_step = atof(contours_step_opt->answer);
    bool use_equalized = false;
    if (equalize_flag->answer)
        use_equalized = true;

    // drawing
    int vect_type;
    get_draw_type(draw_opt->answer, vect_type);
    int draw_threshold = atoi(draw_threshold_opt->answer);
    char* draw_output = NULL;
    if (draw_vector_opt->answer)
        draw_output = draw_vector_opt->answer;

    std::vector<double> draw_x;
    std::vector<double> draw_y;
    std::vector<double> draw_z;
    bool drawing = false;
    unsigned int last_detected_loop_count = 1e6;

    struct Map_info Map_draw;
    struct line_pnts *Points_draw;
    struct line_cats *Cats_draw;
    Points_draw = Vect_new_line_struct();
    Cats_draw = Vect_new_cats_struct();

    Points = Vect_new_line_struct();
    Cats = Vect_new_cats_struct();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(512, 424));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pass (new pcl::PointCloud<pcl::PointXYZRGB>(512, 424));

    struct bound_box bbox;
    struct Cell_head cellhd, window;
    double offset, scale;
    bool region3D = false;

    update_input_region(raster_opt->answer, region_opt->answer, window, offset, region3D);


    K2G k2g(OPENGL);
    k2g.getCloud();
    cloud->sensor_orientation_.w() = 0.0;
    cloud->sensor_orientation_.x() = 1.0;
    cloud->sensor_orientation_.y() = 0.0;
    cloud->sensor_orientation_.z() = 0.0;
    int j = 0;
    // get terminating signals
    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    signal(SIGUSR1, signal_read_new_input);
    while (j < 1) {
        if (signaled == 1) {
            break;
        }
        if (signal_new_input == 1) {
            signal_new_input = 0;
            read_new_input(routput, zrange_min, zrange_max, clip_N, clip_S, clip_E, clip_W,
                           trim_tolerance, angle, zexag, method, numscan, smooth_radius, resolution, use_equalized,
                           window, offset, region3D,
                           color_output, voutput, ply,
                           contours_output, contours_step,
                           vect_type, draw_threshold, draw_output);
        }

        cloud = k2g.getCloud();
        if (!drawing) {
            for (int s = 0; s < numscan - 1; s++)
                *(cloud) += *(k2g.getCloud());
        }

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

        // specify bounding box from center
        if (trim_opt->answer != NULL) {
            clipNSEW(cloud, clip_N, clip_S, clip_E, clip_W);
        }
        // drawing
        if (draw_output) {
            int maxbright = 0;
            int maxbright_idx = 0;
            for (int i=0; i < cloud->points.size(); i++) {
                Eigen::Vector3i rgbv = cloud->points[i].getRGBVector3i();
                int sum = rgbv[0] + rgbv[1] + rgbv[2];
                if (sum > maxbright) {
                    maxbright = sum;
                    maxbright_idx = i;
                }
            }
            if (maxbright >= draw_threshold) {
                drawing = true;
                draw_x.push_back(cloud->points[maxbright_idx].x);
                draw_y.push_back(cloud->points[maxbright_idx].y);
                draw_z.push_back(cloud->points[maxbright_idx].z);
                last_detected_loop_count = 0;
                continue;
            }
            else {
              last_detected_loop_count++;
              if (last_detected_loop_count <= 2) {
                  continue;
                }
            }
        }

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(20);
        sor.setStddevMulThresh(0.5);
        sor.filter(*cloud_filtered_pass);
        cloud_filtered_pass.swap (cloud);

        if (trim_tolerance_opt->answer != NULL) {
            double autoclip_N, autoclip_S, autoclip_E, autoclip_W;
            autotrim(cloud, autoclip_N, autoclip_S, autoclip_E, autoclip_W, trim_tolerance);
            if (autoclip_E > 0 || autoclip_N > 0 || autoclip_S > 0 || autoclip_W > 0)
                trimNSEW(cloud, autoclip_N, autoclip_S, autoclip_E, autoclip_W);
        }

        if (drawing) {
            // get Z scaling
            getMinMax(*cloud, bbox);
            if ((vect_type == GV_AREA && draw_x.size() > 2) ||
                (vect_type == GV_LINE && draw_x.size() > 1) ||
                (vect_type == GV_POINT && draw_x.size() > 0)) {
                save_vector(draw_output, Map_draw, Points_draw, Cats_draw,
                            bbox, window, draw_x, draw_y, draw_z, vect_type, offset, zexag);
            }
            else
                G_warning(_("Tolopogically incorrect vector feature"));
            drawing = false;
            draw_x.clear();
            draw_y.clear();
            draw_z.clear();
            last_detected_loop_count = 1e6;
        }
        if (voutput|| routput || ply || color_output) {
            if (smooth_radius_opt->answer)
                smooth(cloud, smooth_radius);

            // get Z scaling
            getMinMax(*cloud, bbox);
            scale = ((window.north - window.south) / (bbox.N - bbox.S) +
                     (window.east - window.west) / (bbox.E - bbox.W)) / 2;
        }
        // write to vector
        if (voutput|| (routput && strcmp(method, "interpolation") == 0)) {
            double z;
            if (voutput) {
                if (Vect_open_new(&Map, voutput, WITH_Z) < 0)
                    G_fatal_error(_("Unable to create temporary vector map <%s>"), voutput);
            }
            else {
                if (Vect_open_tmp_new(&Map, routput, WITH_Z) < 0)
                    G_fatal_error(_("Unable to create temporary vector map <%s>"), routput);
            }
            for (int i=0; i < cloud->points.size(); i++) {
                Vect_reset_line(Points);
                Vect_reset_cats(Cats);
                if (region3D)
                    z = (cloud->points[i].z + zrange_max) * scale / zexag + offset;
                else
                    z = (cloud->points[i].z - bbox.B) * scale / zexag + offset;
                Vect_append_point(Points, cloud->points[i].x,
                                  cloud->points[i].y,
                                  z);
                Vect_cat_set(Cats, 1, cat);
                Vect_write_line(&Map, GV_POINT, Points, Cats);
            }
            if (strcmp(method, "interpolation") == 0) {
                // interpolate
                Vect_rewind(&Map);
                interpolate(&Map, routput, 20, 2, 50, 40, -1,
                            &bbox, resolution);
            }
            Vect_close(&Map);
        }

        if (routput || color_output) {
            if (routput) {
                if (strcmp(method, "interpolation") != 0) {
                    binning(cloud, routput, &bbox, resolution,
                            scale, zexag, region3D ? -zrange_max : bbox.B, offset, method);
                }
                Rast_get_cellhd(routput, "", &cellhd);
            }
            if (color_output) {
                binning_color(cloud, color_output, &bbox, resolution);
                Rast_get_cellhd(get_color_name(color_output, "r"), "", &cellhd);
            }

            // georeference horizontally
            window.rows = cellhd.rows;
            window.cols = cellhd.cols;
            G_adjust_Cell_head(&window, 1, 1);
            cellhd.north = window.north;
            cellhd.south = window.south;
            cellhd.east = window.east;
            cellhd.west = window.west;
            if (routput)
                Rast_put_cellhd(routput, &cellhd);
            if (color_output) {
                char* output_r = get_color_name(color_output, "r");
                char* output_g = get_color_name(color_output, "g");
                char* output_b = get_color_name(color_output, "b");
                Rast_put_cellhd(output_r, &cellhd);
                Rast_put_cellhd(output_g, &cellhd);
                Rast_put_cellhd(output_b, &cellhd);
            }

            if (contours_output) {
                contours(routput, contours_output, atof(contours_step_opt->answer));
            }
            if (use_equalized) {
                equalized(routput);
            }
        }
        // write to PLY
        if (ply) {
            pcl::PLYWriter writer;
            for (int i=0; i < cloud->points.size(); i++) {
                if (region3D)
                    cloud->points[i].z = (cloud->points[i].z + zrange_max) * scale / zexag + offset;
                else
                    cloud->points[i].z = (cloud->points[i].z - bbox.B) * scale / zexag + offset;
                cloud->points[i].x = (cloud->points[i].x - bbox.W) * scale + window.west;
                cloud->points[i].y = (cloud->points[i].y - bbox.S) * scale + window.south;

            }
            writer.write<pcl::PointXYZRGB>(ply, *cloud, true, true);
        }

        if (!loop_flag->answer)
            j++;
    }

    k2g.shutDown();

    return EXIT_SUCCESS;
}
