#ifndef FEMTODRIVER_H
#define FEMTODRIVER_H

// Point Cloud Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GRASS GIS Includes
extern "C" {
    #include <grass/gis.h>
    #include <grass/glocale.h>
    #undef n_  // Fixing a macro collision with PCL
}

#include <string>
#include <iostream>
#include <tuple>
#include <fstream>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <limits.h>

// Include statements for Orbbec SDK v2
#include "libobsensor/ObSensor.hpp"

// Aspect ratio for all resolution types
#define ASPECT_RATIO (16.0 / 9)

// Femto Color Resolution integration
enum femto_color_resolution_t {
    FEMTO_COLOR_RESOLUTION_ANY = OB_WIDTH_ANY,
    FEMTO_COLOR_RESOLUTION_720P = 720,
    FEMTO_COLOR_RESOLUTION_1080P = 1080,
    FEMTO_COLOR_RESOLUTION_1440P = 1440,
    FEMTO_COLOR_RESOLUTION_2160P = 2160
};

/**
 * Provides functionality for accessing depth and color data from Femto-Bolt depth camera
 */
class FemtoDriver {
public:
    const unsigned int MAX_QUEUE_SIZE = 2;  // Max number of clouds stored in pointCloudQueue
    // Max number of ms to wait for a frame before refreshing
    // It will still hang forever until it gets a non-null thread
    const unsigned int FRAME_WAIT_TIME = 1000;

    /**
     * Default constructor, use the initialize method to init
     */
    FemtoDriver() : running(false) {}

    /**
     * Start the cloud conversion thread with the specified color resolution
     * @param resolution the resolution to use for the color camera
     */
    void initialize(femto_color_resolution_t resolution) {
        // Start the thread function when we first need a cloud
        color_resolution = resolution;
    }

    /**
     * Grabs the most recent frame from the camera and generates a point cloud
     * @param color if you want to include color in the point cloud
     * @param depth2color if you want to include a depth to color mapping in the point cloud,
     * else include a color2depth mapping
     * @return the point cloud with the specified parameters
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(bool color, bool depth2color) {
        // Checking the colors and depth2color for changes
        if (color != global_color || depth2color != global_d2c) {
            // If we call again with the same arguments, then we don't modify the thread
            global_color = color;
            global_d2c = depth2color;
            if (running.load()) {
                std::cout << "Changing parameters, shutting down..." << std::endl;
                shut_down();
            }
        }

        // Starting the thread function with the correct arguments if it's not already going
        if (!running.load()) {
            start_conversion_thread(global_color, global_d2c);
        }

        // Waiting on a new point cloud to finish processing
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;        
        std::unique_lock<std::mutex> lock(queueMutex);
        queueEmpty.wait(lock, [this]{
            return !pointCloudQueue.empty() || !running.load();
        });

        // Grabbing the point cloud from the queue
        if (!pointCloudQueue.empty()) {
            cloud = pointCloudQueue.front();
            pointCloudQueue.pop_front();
        } else {
            throw std::runtime_error("No point cloud found, program terminated early");
        }

        std::cout << "Returning Cloud with size " << cloud->size() << std::endl;
        return cloud;
    }

    /**
     * Kills the converter thread, and shuts down the pipeline
     */
    void shut_down() {
        running.store(false);  // Stopping the thread
        queueEmpty.notify_one(); // Notifying the queues
        if (converter.joinable()) converter.join();
    }

private:
    std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointCloudQueue;  // Queue for storing the point clouds for each type
    std::atomic<bool> running;  // Thread-safe running variable
    std::thread converter;  // The thread for converting all the clouds
    femto_color_resolution_t color_resolution = FEMTO_COLOR_RESOLUTION_ANY;  // Resolution for the color camera

    // Condition variables for enforcing mutual exclusion, overdraft, and overwrite respectively
    std::mutex queueMutex;
    std::condition_variable queueEmpty;

    // Variables to keep track of the point cloud type
    bool global_color = true;
    bool global_d2c = true;

    /**
     * Runs a thread that converts frames from the Femto-Bolt to point clouds and stores them
     * in their respective queues
     */
    void start_conversion_thread(bool color, bool depth2color) {
        running.store(true);

        std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>();

        // Pipeline and PCF configuration logic
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

        // Configure which streams to enable or disable for the Pipeline by creating a Config
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        // Creating the color stream
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
        try {
            // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
            auto colorProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
            if (colorProfiles) {
                auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
                colorProfile = profile->as<ob::VideoStreamProfile>();
            }
            // Creating the video stream with the desired resolution
            config->enableVideoStream(OB_SENSOR_COLOR, get_camera_width(color_resolution), color_resolution, OB_FPS_ANY,
                                        OB_FORMAT_RGB);
        } catch(ob::Error &e) {
            config->setAlignMode(ALIGN_DISABLE);
            std::cerr << "Current device does not support color sensor!" << std::endl;
        }

        // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
        std::shared_ptr<ob::StreamProfileList> depthProfileList;
        OBAlignMode alignMode = ALIGN_DISABLE;
        if (colorProfile) {
            // Try find supported depth to color align hardware mode profile
            depthProfileList = pipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
            } else {
                // Try find supported depth to color align software mode profile
                depthProfileList = pipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if (depthProfileList->count() > 0) {
                    // I'm pretty sure that only this mode is supported on the Femto-Bolt
                    alignMode = ALIGN_D2C_SW_MODE;
                }
            }

            try {
                // Enable frame synchronization
                pipe->enableFrameSync();
            } catch (ob::Error &e) {
                std::cerr << "Current device is not support frame sync!" << std::endl;
            }
        } else {
            depthProfileList = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
        }

        // Creating the depth stream
        if (depthProfileList->count() > 0) {
            std::shared_ptr<ob::StreamProfile> depthProfile;
            try {
                // Select the profile with the same frame rate as color.
                if (colorProfile) {
                    depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY,  OB_FORMAT_ANY, colorProfile->fps());
                }
            } catch(...) {
                depthProfile = nullptr;
            }

            if (!depthProfile) {
                // If no matching profile is found, select the default profile.
                depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
            }
            config->enableStream(depthProfile);
        }
        config->setAlignMode(alignMode);

        // Essential for frame alignment
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

        // Starting the pipeline
        pipe->start(config);
        
        // Starting the thread
        std::cout << "Starting thread function with Color: " << color << " and D2C: " << depth2color << std::endl;
        converter = std::thread(&FemtoDriver::threadFunction, this, pipe, color, depth2color);
    }

    /**
     * Reads frames from the Femto-Bolt and processes them into point clouds
     * @param pipeline the pipeline to read frames from
     * @param color if you want to include color in the final point cloud
     * @param depth2color if you want to perform depth-to-color alignment, else perform color-to-depth alignment
     */
    void threadFunction(std::shared_ptr<ob::Pipeline> pipeline, bool color, bool depth2color) {
        ob::PointCloudFilter pointCloudFilter;
        std::shared_ptr<ob::Align> align;
        pointCloudFilter.setCameraParam(pipeline->getCameraParam());
        float depthValueScale;

        // Type-specific initialization
        if (color) {
            if (depth2color) {
                // D2C
                align = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            } else {
                // C2D
                align = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
            }
            pointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT);
        } else {
            // Depth Only - no alignment
            pointCloudFilter.setCreatePointFormat(OB_FORMAT_POINT);
        }

        while (running.load()) {
            // Waiting for a non-null frameset
            auto fs = pipeline->waitForFrames(FRAME_WAIT_TIME);
            if (fs == nullptr) continue;

            // Frames should be synchronized here
            if (fs->depthFrame() != nullptr && fs->colorFrame() != nullptr) {
                // Setting depth scale for point cloud filter
                depthValueScale = fs->depthFrame()->getValueScale();
                pointCloudFilter.setPositionDataScaled(depthValueScale);

                // Alignment processing - not for depth
                std::shared_ptr<ob::Frame> point_cloud_frame;
                if (color) {
                    std::shared_ptr<ob::Frame> aligned_frame = align->process(fs);
                    point_cloud_frame = pointCloudFilter.process(aligned_frame);
                } else {
                    point_cloud_frame = pointCloudFilter.process(fs);
                }

                // Filtering and enqueuing
                if (point_cloud_frame != nullptr) {
                    // Converting to point cloud outside the critical section
                    auto temp_cloud = frame_to_point_cloud(point_cloud_frame, color);

                    // Critical section, locking and removing a stale point cloud
                    std::unique_lock<std::mutex> lock(queueMutex);
                    if (pointCloudQueue.size() >= MAX_QUEUE_SIZE) {
                        pointCloudQueue.pop_front();
                    }

                    // Inserting the point cloud now that we have space, then notify the consumer
                    pointCloudQueue.push_back(temp_cloud);
                    queueEmpty.notify_one();
                } else {
                    throw std::runtime_error("Processed C2D Cloud was NULL");
                }
            }
        }

        // Shutting down the pipeline before exiting
        pipeline->stop();
    }

    /**
     * Copies the contents of the frame into a point cloud and returns it
     * @param frame the frame to convert into a point cloud
     * @return the completed point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_to_point_cloud(const std::shared_ptr<ob::Frame>& ob_frame, bool hasColor) {
        // If the frame isn't defined or if it isn't a PointsFrame
        if (ob_frame == nullptr || ob_frame->type() != OB_FRAME_POINTS) {
            return nullptr;
        }

        // Grabbing the points and the data from the Frame
        auto ob_points = ob_frame->as<ob::PointsFrame>();
        OBColorPoint *colorPoints;
        OBPoint *points;
        int length = 0;

        // Depending on if we want color, use a different type of OBPoint
        if (hasColor) {
            colorPoints = (OBColorPoint *) ob_points->data();
            length = ob_points->dataSize() / sizeof(OBColorPoint);
        } else {
            points = (OBPoint *) ob_points->data();
            length = ob_points->dataSize() / sizeof(OBPoint);
        }

        // Initializing the point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        static const double epsilon = 1e-6;  // Threshold for finding invalid points

        // Copying the Frame data into the new point cloud
        if (hasColor) {
            for (int i = 0; i < length; i++) {
                if (std::abs(colorPoints[i].x) > epsilon && std::abs(colorPoints[i].y) > epsilon && std::abs(colorPoints[i].z) > epsilon) {
                    // Adjusting the reference frame and copying the color cloud
                    pcl::PointXYZRGB point;
                    point.x = -colorPoints[i].x / 1000.0;
                    point.y = colorPoints[i].y / 1000.0 * ASPECT_RATIO;
                    point.z = -colorPoints[i].z / 1000.0;
                    point.r = static_cast<std::uint8_t>(colorPoints[i].r);
                    point.g = static_cast<std::uint8_t>(colorPoints[i].g);
                    point.b = static_cast<std::uint8_t>(colorPoints[i].b);
                    pcl_cloud->points.push_back(point);
                }
            }
        } else {
            for (int i = 0; i < length; i++) {
                if (std::abs(points[i].x) > epsilon && std::abs(points[i].y) > epsilon && std::abs(points[i].z) > epsilon) {
                    // Adjusting the reference frame and copying the depth cloud
                    pcl::PointXYZRGB point;
                    point.x = -points[i].x / 1000.0;
                    point.y = points[i].y / 1000.0 * ASPECT_RATIO;
                    point.z = -points[i].z / 1000.0;
                    point.r = static_cast<std::uint8_t>(0);
                    point.g = static_cast<std::uint8_t>(0);
                    point.b = static_cast<std::uint8_t>(0);
                    pcl_cloud->points.push_back(point);
                } 
            }
        }

        // Resizing the point cloud, if necessary
        if (pcl_cloud->points.size() != pcl_cloud->height * pcl_cloud->width) {
            pcl_cloud->height = static_cast<std::uint32_t>(1);  // Height is 1 for an unorganized cloud
            pcl_cloud->width = static_cast<std::uint32_t>(pcl_cloud->points.size());
        }
        // Cloud should be dense since we removed all the invalid points
        pcl_cloud->is_dense = true;

        return pcl_cloud;
    }

    /**
     * Gets the camera width based on the height using a 16x9 ratio
     * @param camera_height the height of the camera in pixels
     */
    int get_camera_width(int camera_height) {
        return (camera_height * 16) / 9;
    }
};

#endif // FEMTODRIVER_H