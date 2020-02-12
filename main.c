/*
 ****************************************************************************
 *
 * MODULE:       r.in.kinect
 * AUTHOR(S):    Anna Petrasova
 * PURPOSE:      Import data from Azure Kinect DK
 * COPYRIGHT:    (C) 2020 by the GRASS Development Team
 *
 *               This program is free software under the GNU General
 *               Public License (>=v2). Read the file COPYING that
 *               comes with GRASS for details.
 *
 *****************************************************************************/

#include "utils.h"
#include "binning.h"

#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>
#include <grass/glocale.h>

#include <k4a/k4a.h>

#include <stdlib.h>



int main(int argc, char **argv)
{
    struct GModule *module;
    struct Option *routput_opt, *raster_opt, *resolution_opt;


    G_gisinit(argv[0]);

    module = G_define_module();
    G_add_keyword(_("3D"));
    G_add_keyword(_("scan"));
    G_add_keyword(_("point cloud"));
    module->description = _("Imports data from Azure Kinect DK");

    routput_opt = G_define_standard_option(G_OPT_R_OUTPUT);
    routput_opt->guisection = _("Output");
    routput_opt->required = YES;

    resolution_opt = G_define_option();
    resolution_opt->key = "resolution";
    resolution_opt->type = TYPE_DOUBLE;
    resolution_opt->required = NO;
    resolution_opt->answer = "0.002";
    resolution_opt->label = _("Raster resolution");
    resolution_opt->description = _("Recommended values between 0.001-0.003");
    resolution_opt->guisection = _("Output");

    raster_opt = G_define_standard_option(G_OPT_R_MAP);
    raster_opt->key = "raster";
    raster_opt->required = NO;
    raster_opt->multiple = NO;
    raster_opt->description = _("Match resulting raster to this raster map");
    raster_opt->guisection = _("Georeferencing");

    if (G_parser(argc, argv))
        exit(EXIT_FAILURE);

    // initailization of variables
    double resolution = 0.002;
    if (resolution_opt->answer)
        resolution = atof(resolution_opt->answer);
    char* routput = NULL;
    if (routput_opt->answer)
        routput = routput_opt->answer;


    struct bound_box bbox;
    struct Cell_head cellhd, window;
    double offset, scale;
    
    //k4a
    // from https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/streaming/main.c
    k4a_device_t device = NULL;        
    k4a_capture_t capture = NULL;
    k4a_transformation_t transformation = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t point_cloud_image = NULL;
    k4a_calibration_t calibration;
    unsigned device_count = k4a_device_get_installed_count();
    if (device_count == 0)
        G_fatal_error(_("No Kinect device found"));
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
        G_fatal_error(_("Failed to open Kinect device"));
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode,
                                   config.color_resolution, &calibration))
        G_fatal_error(_("Failed to get calibration"));

    transformation = k4a_transformation_create(&calibration);
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                     calibration.depth_camera_calibration.resolution_width,
                     calibration.depth_camera_calibration.resolution_height,
                     calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                     &point_cloud_image);
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
        G_fatal_error(_("Failed to start Kinect"));

    update_input_region(raster_opt->answer, &window, &offset);
    
    // get cloud
    switch (k4a_device_get_capture(device, &capture, 2000))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        G_warning("Timed out waiting for a capture\n");
        //continue;
        G_fatal_error(_("Failed to capture"));
    case K4A_WAIT_RESULT_FAILED:
        G_fatal_error(_("Failed to capture"));
    }
    // get depth
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
        G_fatal_error(_("Failed to get depth image from capture"));
    if (K4A_RESULT_SUCCEEDED != 
            k4a_transformation_depth_image_to_point_cloud(transformation,
                                                          depth_image,
                                                          K4A_CALIBRATION_TYPE_DEPTH,
                                                          point_cloud_image))
        G_fatal_error(_("Failed to compute point cloud"));
    
    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    unsigned npoints = k4a_image_get_width_pixels(point_cloud_image) * 
            k4a_image_get_height_pixels(point_cloud_image);
    
    // filter nans
    for (int i = 0; i < npoints; i++)
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
            continue;
    for (int i = 0; i < npoints; i++) {
        if (i % 1000){
            printf("bbox %d %f\n", i, point_cloud_data[i].xyz.x);
            fflush(stdout);}
    }
    get_min_max(point_cloud_data, npoints, &bbox);
    scale = ((window.north - window.south) / (bbox.N - bbox.S) +
             (window.east - window.west) / (bbox.E - bbox.W)) / 2;
    printf("bbox %d %f %f %f %f\n", npoints, bbox.N, bbox.S, bbox.E, bbox.W);
    fflush(stdout);
    
    // binning here
    binning(point_cloud_data, npoints, routput, &bbox, resolution, scale, 1, bbox.B, offset, "mean");
    Rast_get_cellhd(routput, "", &cellhd);
    
    k4a_image_release(depth_image);
    k4a_image_release(point_cloud_image);
    
    // georeference horizontally
    window.rows = cellhd.rows;
    window.cols = cellhd.cols;
    G_adjust_Cell_head(&window, 1, 1);
    cellhd.north = window.north;
    cellhd.south = window.south;
    cellhd.east = window.east;
    cellhd.west = window.west;
    cellhd.ns_res = window.ns_res;
    cellhd.ew_res = window.ew_res;
    Rast_put_cellhd(routput, &cellhd);
    
    k4a_capture_release(capture);
    if (device)
        k4a_device_close(device);


    return EXIT_SUCCESS;
}
