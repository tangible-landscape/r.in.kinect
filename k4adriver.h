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
#include <tuple>


class K4ADriver {
public:
    K4ADriver()
    {
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.synchronized_images_only = true;
    }
    void initialize(k4a_depth_mode_t depth_mode,
                    k4a_color_resolution_t color_resolution)
    {
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(bool color, bool depth2color)
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
        // get depth in any case
        point_cloud_image = 0;
        color_image = 0;
        transformed_depth_image = 0;
        transformed_color_image = 0;
        point_cloud_image = 0;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (color)
            if (depth2color)
                cloud = prepare_cloud_RGBD_D2C();
            else
                cloud = prepare_cloud_RGBD_C2D();
        else
            cloud = prepare_cloud_D();

        release();

        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepare_cloud_D()
    {
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
        return create_cloud(point_cloud_image, 0);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepare_cloud_RGBD_C2D()
    {
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0) {
            release();
            throw std::runtime_error("Failed to get depth image from capture");
        }
        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0) {
            release();
            throw std::runtime_error("Failed to get color image from capture");
        }
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                         k4a_image_get_width_pixels(depth_image),
                         k4a_image_get_height_pixels(depth_image),
                         k4a_image_get_width_pixels(depth_image) * 4 * (int)sizeof(uint8_t),
                         &transformed_color_image);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         k4a_image_get_width_pixels(depth_image),
                         k4a_image_get_height_pixels(depth_image),
                         k4a_image_get_width_pixels(depth_image) * 3 * (int)sizeof(int16_t),
                         &point_cloud_image);
        if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_color_image_to_depth_camera(transformation,
                                                               depth_image,
                                                               color_image,
                                                               transformed_color_image)) {
            release();
            throw std::runtime_error("Failed to compute transformed depth image");
        }
        if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_depth_image_to_point_cloud(transformation,
                                                              depth_image,
                                                              K4A_CALIBRATION_TYPE_DEPTH,
                                                              point_cloud_image)) {
            release();
            throw std::runtime_error("Failed to compute point cloud");
        }
        return create_cloud(point_cloud_image, transformed_color_image);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepare_cloud_RGBD_D2C()
    {
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0) {
            release();
            throw std::runtime_error("Failed to get depth image from capture");
        }
        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0) {
            release();
            throw std::runtime_error("Failed to get color image from capture");
        }
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                         k4a_image_get_width_pixels(color_image),
                         k4a_image_get_height_pixels(color_image),
                         k4a_image_get_width_pixels(color_image) * (int)sizeof(uint16_t),
                         &transformed_depth_image);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         k4a_image_get_width_pixels(color_image),
                         k4a_image_get_height_pixels(color_image),
                         k4a_image_get_width_pixels(color_image) * 3 * (int)sizeof(int16_t),
                         &point_cloud_image);
        if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_depth_image_to_color_camera(transformation,
                                                               depth_image,
                                                               transformed_depth_image)) {
            release();
            throw std::runtime_error("Failed to compute transformed depth image");
        }
        if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_depth_image_to_point_cloud(transformation,
                                                              transformed_depth_image,
                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                              point_cloud_image)) {
            release();
            throw std::runtime_error("Failed to compute point cloud");
        }
        return create_cloud(point_cloud_image, color_image);
    }
    void release()
    {
        if (color_image)
            k4a_image_release(color_image);
        if (depth_image)
            k4a_image_release(depth_image);
        if (point_cloud_image)
            k4a_image_release(point_cloud_image);
        if (transformed_color_image)
            k4a_image_release(transformed_color_image);
        if (transformed_depth_image)
            k4a_image_release(transformed_depth_image);
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
    k4a_image_t color_image;
    k4a_image_t point_cloud_image;
    k4a_image_t transformed_depth_image;
    k4a_image_t transformed_color_image;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr create_cloud(k4a_image_t input_point_cloud,
                                                        k4a_image_t input_color_image)
    {
        int16_t *point_cloud_data = (int16_t *)(void *)k4a_image_get_buffer(input_point_cloud);
        unsigned width = k4a_image_get_width_pixels(input_point_cloud);
        unsigned height = k4a_image_get_height_pixels(input_point_cloud);
        uint8_t *color_image_data;
        if (input_color_image)
            color_image_data = k4a_image_get_buffer(input_color_image);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
        std::size_t j = 0;
        for (std::size_t i = 0; i < width * height; ++i) {
            if (point_cloud_data[3 * i + 0] == 0 ||
                    point_cloud_data[3 * i + 1] == 0 ||
                    point_cloud_data[3 * i + 2] == 0)
                continue;
            if (input_color_image && color_image_data[4 * i + 0] == 0 &&
                    color_image_data[4 * i + 1] == 0 &&
                    color_image_data[4 * i + 2] == 0 &&
                    color_image_data[4 * i + 3] == 0)
                continue;
            cloud->points[j].x = -point_cloud_data[3 * i + 0] / 1000.;
            cloud->points[j].y = point_cloud_data[3 * i + 1] / 1000.;
            cloud->points[j].z = -point_cloud_data[3 * i + 2] / 1000.;
            cloud->points[j].b = input_color_image ? color_image_data[4 * i + 0] : 0;
            cloud->points[j].g = input_color_image ? color_image_data[4 * i + 1] : 0;
            cloud->points[j].r = input_color_image ? color_image_data[4 * i + 2] : 0;
            j++;
        }
        if (j != width * height)
            cloud->points.resize(j);
        cloud->height = 1;
        cloud->width = static_cast<std::uint32_t>(j);
        cloud->is_dense = true;
        return cloud;
    }
};

#endif // K4ADRIVER_H
