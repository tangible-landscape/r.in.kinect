#ifndef REALSENSEDRIVER_H
#define REALSENSEDRIVER_H

extern "C" {
    #include <grass/gis.h>
    #include <grass/glocale.h>
}
// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <iostream>
#include <tuple>


class RealSenseDriver {
public:
    RealSenseDriver()
    {
//        config.camera_fps = K4A_FRAMES_PER_SECOND_5;

    }
    void initialize()
    {
        // Start streaming with default recommended configuration
        pipe.start();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(bool use_color)
    {
        // Wait for frames from the camera to settle
       for (int i = 0; i < 30; i++) {
           auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
       }

       // Capture a single frame and obtain depth + RGB values from it
       auto frames = pipe.wait_for_frames();
       auto depth = frames.get_depth_frame();
       auto RGB = frames.get_color_frame();

       // Map Color texture to each point
       pc.map_to(RGB);

       // Generate Point Cloud
       auto points = pc.calculate(depth);
       auto cloud = create_cloud(points);
       return cloud;
    }


    void release(){}
    void shut_down(){}

private:
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr create_cloud(const rs2::points& points)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        for (auto& p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            p.r = 0;
            p.g = 0;
            p.b = 0;
            ptr++;
        }

        return cloud;
    }
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr create_cloud(k4a_image_t input_point_cloud,
//                                                        k4a_image_t input_color_image)
//    {
//        int16_t *point_cloud_data = (int16_t *)(void *)k4a_image_get_buffer(input_point_cloud);
//        unsigned width = k4a_image_get_width_pixels(input_point_cloud);
//        unsigned height = k4a_image_get_height_pixels(input_point_cloud);
//        uint8_t *color_image_data;
//        if (input_color_image)
//            color_image_data = k4a_image_get_buffer(input_color_image);
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
//        std::size_t j = 0;
//        for (std::size_t i = 0; i < width * height; ++i) {
//            if (point_cloud_data[3 * i + 0] == 0 ||
//                    point_cloud_data[3 * i + 1] == 0 ||
//                    point_cloud_data[3 * i + 2] == 0)
//                continue;
//            if (input_color_image && color_image_data[4 * i + 0] == 0 &&
//                    color_image_data[4 * i + 1] == 0 &&
//                    color_image_data[4 * i + 2] == 0 &&
//                    color_image_data[4 * i + 3] == 0)
//                continue;
//            cloud->points[j].x = -point_cloud_data[3 * i + 0] / 1000.;
//            cloud->points[j].y = point_cloud_data[3 * i + 1] / 1000.;
//            cloud->points[j].z = -point_cloud_data[3 * i + 2] / 1000.;
//            cloud->points[j].b = input_color_image ? color_image_data[4 * i + 0] : 0;
//            cloud->points[j].g = input_color_image ? color_image_data[4 * i + 1] : 0;
//            cloud->points[j].r = input_color_image ? color_image_data[4 * i + 2] : 0;
//            j++;
//        }
//        if (j != width * height)
//            cloud->points.resize(j);
//        cloud->height = 1;
//        cloud->width = static_cast<std::uint32_t>(j);
//        cloud->is_dense = true;
//        return cloud;
//    }
};

#endif // REALSENSEDRIVER_H
