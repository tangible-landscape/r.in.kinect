#ifndef K4ADRIVER_H
#define K4ADRIVER_H

extern "C" {
    #include <grass/gis.h>
    #include <grass/glocale.h>
}
#include <k4a/k4a.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <iostream>

class K4ADriver {
public:
    K4ADriver(k4a_depth_mode_t depth_mode,
              k4a_color_resolution_t color_resolution)
    {
        qnan_ = std::numeric_limits<float>::quiet_NaN();
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
        config.depth_mode = depth_mode;
        config.color_resolution = color_resolution;
        
        unsigned device_count = k4a_device_get_installed_count();
        if (device_count == 0)
            G_fatal_error(_("No Kinect device found"));
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
            G_fatal_error(_("Failed to open Kinect device"));
        if (K4A_RESULT_SUCCEEDED !=
            k4a_device_get_calibration(device, config.depth_mode,
                                       config.color_resolution, &calibration))
            G_fatal_error(_("Failed to get calibration"));
        transformation = k4a_transformation_create(&calibration);
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
            G_fatal_error(_("Failed to start Kinect"));
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud()
    {
        switch (k4a_device_get_capture(device, &capture, 2000))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            release();
            throw std::runtime_error("Timed out waiting for a capture");
        case K4A_WAIT_RESULT_FAILED:
            release();
            throw std::runtime_error("Failed to capture");
        }
        // get depth
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0) {
            release();
            throw std::runtime_error("Failed to get depth image from capture");
        }
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         k4a_image_get_width_pixels(depth_image),
                         k4a_image_get_height_pixels(depth_image),
                         k4a_image_get_width_pixels(depth_image) * 3 * (int)sizeof(int16_t),
                         &point_cloud_image);
        if (K4A_RESULT_SUCCEEDED != 
                k4a_transformation_depth_image_to_point_cloud(transformation,
                                                              depth_image,
                                                              K4A_CALIBRATION_TYPE_DEPTH,
                                                              point_cloud_image)) {
            release();
            throw std::runtime_error("Failed to compute point cloud");
        }
        int16_t *point_cloud_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
        unsigned width = k4a_image_get_width_pixels(point_cloud_image);
        unsigned height = k4a_image_get_height_pixels(point_cloud_image);
        G_warning("width %d height %d", width, height);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
        pcl::PointXYZRGB * itP = &cloud->points[0];
        bool is_dense = true;
        for (int i = 0; i < width * height; ++i, ++itP) {
            if (point_cloud_data[3 * i + 0] == 0 ||
                    point_cloud_data[3 * i + 1] == 0 ||
                    point_cloud_data[3 * i + 2] == 0) {
                itP->x = qnan_;
                itP->y = qnan_;
                itP->z = qnan_;
                is_dense = false;
            } else {
                itP->x = -point_cloud_data[3 * i + 0] / 1000.;
                itP->y = point_cloud_data[3 * i + 1] / 1000.;
                itP->z = -point_cloud_data[3 * i + 2] / 1000.;
            }
            itP->b = 0;
            itP->g = 0;
            itP->r = 0;
        }
        cloud->is_dense = is_dense;
        release();
        return cloud;
    }
    void release()
    {
        if (depth_image)
            k4a_image_release(depth_image);
        if (point_cloud_image)
            k4a_image_release(point_cloud_image);
        if (capture)
            k4a_capture_release(capture);
    }
    void shut_down()
    {
        if (transformation)
            k4a_transformation_destroy(transformation);
        if (device)
            k4a_device_close(device);
    }
    
private:
    k4a_device_t device;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation;
    k4a_capture_t capture;
    k4a_image_t depth_image;
    k4a_image_t point_cloud_image;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    float qnan_;
};

#endif // K4ADRIVER_H
