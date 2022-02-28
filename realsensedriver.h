/* Adapted from RealSense SDK examples:
 * https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl/rs-pcl.cpp
 */
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


using namespace std;


class RealSenseDriver {
public:
    RealSenseDriver(){}
    void initialize()
    {
        // Start streaming with default recommended configuration
        pipe.start();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(bool use_color)
    {
        // Wait for frames from the camera to settle
        if (use_color) {
            for (int i = 0; i < 30; i++) {
                auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
            }
        }
       // Capture a single frame and obtain depth + RGB values from it
       auto frames = pipe.wait_for_frames();
       auto depth = frames.get_depth_frame();
       if (use_color) {
           auto RGB = frames.get_color_frame();
           // Map Color texture to each point
           pc.map_to(RGB);
           auto points = pc.calculate(depth);
           return create_colored_cloud(points, RGB);
       }
       else {
           auto points = pc.calculate(depth);
           return create_cloud(points);
       }

    }


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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr create_colored_cloud(const rs2::points& points, const rs2::video_frame& color){

        // Object Declaration (Point Cloud)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
        std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

        //================================
        // PCL Cloud Object Configuration
        //================================
        // Convert data captured from Realsense camera to Point Cloud
        auto sp = points.get_profile().as<rs2::video_stream_profile>();

        cloud->width  = static_cast<uint32_t>( sp.width()  );
        cloud->height = static_cast<uint32_t>( sp.height() );
        cloud->is_dense = false;
        cloud->points.resize( points.size() );

        auto Texture_Coord = points.get_texture_coordinates();
        auto Vertex = points.get_vertices();

        // Iterating through all points and setting XYZ coordinates
        // and RGB values
        for (int i = 0; i < points.size(); i++)
        {
            //===================================
            // Mapping Depth Coordinates
            // - Depth data stored as XYZ values
            //===================================
            cloud->points[i].x = Vertex[i].x;
            cloud->points[i].y = Vertex[i].y;
            cloud->points[i].z = Vertex[i].z;

            // Obtain color texture for specific point
            RGB_Color = RGB_Texture(color, Texture_Coord[i]);

            // Mapping Color (BGR due to Camera Model)
            cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
            cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
            cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

        }

       return cloud; // PCL RGB Point Cloud generated
    }

    std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
    {
        // Get Width and Height coordinates of texture
        int width  = texture.get_width();  // Frame width in pixels
        int height = texture.get_height(); // Frame height in pixels

        // Normals to Texture Coordinates conversion
        int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
        int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

        int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
        int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
        int Text_Index =  (bytes + strides);

        const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

        // RGB components to save in tuple
        int NT1 = New_Texture[Text_Index];
        int NT2 = New_Texture[Text_Index + 1];
        int NT3 = New_Texture[Text_Index + 2];

        return std::tuple<int, int, int>(NT1, NT2, NT3);
    }
};

#endif // REALSENSEDRIVER_H
