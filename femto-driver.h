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
    const unsigned int MAX_QUEUE_SIZE = 2;  // Max number of clouds stored in depthCloudQueue
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
        // If running, shut down before proceeding
        if (running.load()) {
            std::cout << "Thread already active, shutting down..." << std::endl;
            shut_down();
        }
        // Start the cloud processing thread with the specific cloud type
        if (!running.load()) {
            convertClouds(color, depth2color);
        }

        // Checking cloud options
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        if (color) {
            if (depth2color) {
                std::cout << "Waiting on D2C cloud!" << std::endl;
                // Waiting on a new depth2color point cloud
                std::unique_lock<std::mutex> lock(d2cQueueMutex);
                d2cQueueEmpty.wait(lock, [this]{
                    return !d2cCloudQueue.empty() || !running.load();
                });

                // Grabbing a depth2color point cloud from the queue
                if (!d2cCloudQueue.empty()) {
                    cloud = d2cCloudQueue.front();
                    d2cCloudQueue.pop_front();
                } else {
                    throw std::runtime_error("No D2C cloud found, program terminated early");
                }
            } else {
                // Waiting on a new color2depth point cloud
                std::unique_lock<std::mutex> lock(c2dQueueMutex);
                c2dQueueEmpty.wait(lock, [this]{
                    return !c2dCloudQueue.empty() || !running.load();
                });

                // Grabbing a new color2depth point cloud from the queue
                if (!c2dCloudQueue.empty()) {
                    cloud = c2dCloudQueue.front();
                    c2dCloudQueue.pop_front();
                } else {
                    throw std::runtime_error("No C2D cloud found, program terminated early");
                }
            }
        } else {
            // Waiting on termination or a new depth point cloud
            std::unique_lock<std::mutex> lock(depthQueueMutex);
            depthQueueEmpty.wait(lock, [this] {
                return !depthCloudQueue.empty() || !running.load();
            });

            // Grabbing a new depth point cloud from the queue
            if (!depthCloudQueue.empty()) {
                cloud = depthCloudQueue.front();
                depthCloudQueue.pop_front();
            } else {
                std::runtime_error("No depth cloud found; program terminated early");
            }
        }

        return cloud;
    }

    /**
     * Deprecated, do not call
     */
    void release() {
        throw std::runtime_error("Unimplemented Method Exception");
    }

    /**
     * Kills the converter thread, and shuts down the pipeline
     */
    void shut_down() {
        running.store(false);  // Stopping the thread
        depthQueueEmpty.notify_all();  // notifying all the threads
        c2dQueueEmpty.notify_all();
        d2cQueueEmpty.notify_all();
        if (converter.joinable()) converter.join();
    }

private:
    // Mutexes for locking the point cloud queues
    std::mutex depthQueueMutex;
    std::mutex d2cQueueMutex;
    std::mutex c2dQueueMutex;
    // Queues for storing the point clouds for each type
    std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> depthCloudQueue;
    std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> d2cCloudQueue;
    std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> c2dCloudQueue;
    // Conditions for signaling when a cloud enters the queue
    std::condition_variable depthQueueEmpty;
    std::condition_variable d2cQueueEmpty;
    std::condition_variable c2dQueueEmpty;

    std::atomic<bool> running;  // Thread-safe running variable
    std::thread converter;  // The thread for converting all the clouds

    femto_color_resolution_t color_resolution = FEMTO_COLOR_RESOLUTION_ANY;  // Resolution for the color camera

    /**
     * Runs a thread that converts frames from the Femto-Bolt to point clouds and stores them
     * in their respective queues
     */
    void convertClouds(bool color, bool depth2color) {
        std::cout << "Starting convert clouds with " << color << " " << depth2color << std::endl;
        running.store(true);

        std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>();

        // Pipeline and PCF configuration logic
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

        // Configure which streams to enable or disable for the Pipeline by creating a Config
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        // Creating the color stream
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
        try {
            // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
            auto colorProfiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            if (colorProfiles) {
                auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
                colorProfile = profile->as<ob::VideoStreamProfile>();
            }
            // Creating the video stream with the desired resolution
            config->enableVideoStream(OB_SENSOR_COLOR, getCameraWidth(color_resolution), color_resolution, OB_FPS_ANY,
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
            depthProfileList = pipeline->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
            } else {
                // Try find supported depth to color align software mode profile
                depthProfileList = pipeline->getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if (depthProfileList->count() > 0) {
                    // I'm pretty sure that only this mode is supported on the Femto-Bolt
                    alignMode = ALIGN_D2C_SW_MODE;
                }
            }

            try {
                // Enable frame synchronization
                pipeline->enableFrameSync();
            } catch (ob::Error &e) {
                std::cerr << "Current device is not support frame sync!" << std::endl;
            }
        } else {
            depthProfileList = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
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
        pipeline->start(config);

        std::cout << "Finished initialization" << std::endl;

        // Defining the specific thread function
        std::function<void(std::shared_ptr<ob::Pipeline>)> convertFunction;
        if (color) {
            if (depth2color) {
                convertFunction = [this](std::shared_ptr<ob::Pipeline> pipeline) {
                    std::cout << "D2C Thread Started" << std::endl;
                    // Depth2Color Thread initialization
                    std::shared_ptr<ob::Align> align = std::make_shared<ob::Align>(OB_STREAM_COLOR);
                    ob::PointCloudFilter pointCloudFilter;
                    pointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                    pointCloudFilter.setCameraParam(pipeline->getCameraParam());
                    float depthValueScale;

                    while (running.load()) {
                        std::cout << "Waiting for frames" << std::endl;
                        // Waiting for a non-null frameset
                        auto fs = pipeline->waitForFrames(FRAME_WAIT_TIME);
                        if (fs == nullptr) continue;

                        // Frames should be synchronized here
                        std::cout << "Got non-null frame" << std::endl;
                        if (fs->depthFrame() != nullptr && fs->colorFrame() != nullptr) {
                            // Setting depth scale for point cloud filter
                            depthValueScale = fs->depthFrame()->getValueScale();
                            std::cout << "done with value scaling" << std::endl;
                            pointCloudFilter.setPositionDataScaled(depthValueScale);

                            // Alignment processing
                            std::cout << "Aligning D2C" << std::endl;
                            std::shared_ptr<ob::Frame> d2c_aligned = align->process(fs);
                            std::cout << "Processing D2C" << std::endl;
                            std::shared_ptr<ob::Frame> d2c_cloud = pointCloudFilter.process(d2c_aligned);
                            if (d2c_cloud != nullptr) {
                                // Locking the queue to insert a new depth2color point cloud, then notifying
                                std::lock_guard<std::mutex> lock(d2cQueueMutex);
                                if (d2cCloudQueue.size() >= MAX_QUEUE_SIZE) {
                                    d2cCloudQueue.pop_front();
                                }
                                std::cout << "Pushing D2C" << std::endl;
                                d2cCloudQueue.push_back(convertFrameToPointCloud(d2c_cloud, true));
                                d2cQueueEmpty.notify_one();
                            } else {
                                throw std::runtime_error("Processed D2C Cloud was NULL");
                            }
                        }
                    }

                    pipeline->stop();
                };
            } else {
                convertFunction = [this](std::shared_ptr<ob::Pipeline> pipeline) {
                    std::cout << "C2D Thread Started" << std::endl;
                    // Color2Depth Thread initialization
                    std::shared_ptr<ob::Align> align = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
                    ob::PointCloudFilter pointCloudFilter;
                    pointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                    pointCloudFilter.setCameraParam(pipeline->getCameraParam());
                    float depthValueScale;

                    while (running.load()) {
                        // Waiting for a non-null frameset
                        auto fs = pipeline->waitForFrames(FRAME_WAIT_TIME);
                        if (fs == nullptr) continue;

                        // Frames should be synchronized here
                        if (fs->depthFrame() != nullptr && fs->colorFrame() != nullptr) {
                            // Setting depth scale for point cloud filter
                            depthValueScale = fs->depthFrame()->getValueScale();
                            pointCloudFilter.setPositionDataScaled(depthValueScale);

                            // Alignment processing
                            std::shared_ptr<ob::Frame> c2d_aligned = align->process(fs);
                            pointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                            std::shared_ptr<ob::Frame> c2d_cloud = pointCloudFilter.process(c2d_aligned);
                            if (c2d_cloud != nullptr) {
                                // Locking the queue to insert a new color2depth point cloud, then notifying
                                std::lock_guard<std::mutex> lock(c2dQueueMutex);
                                if (c2dCloudQueue.size() >= MAX_QUEUE_SIZE) {
                                    c2dCloudQueue.pop_front();
                                }
                                c2dCloudQueue.push_back(convertFrameToPointCloud(c2d_cloud, true));
                                c2dQueueEmpty.notify_one();
                            } else {
                                throw std::runtime_error("Processed C2D Cloud was NULL");
                            }
                        }
                    }

                    pipeline->stop();
                };
            }
        } else {
            convertFunction = [this](std::shared_ptr<ob::Pipeline> pipeline) {
                std::cout << "Starting Depth thread" << std::endl;
                // Depth Thread initialization
                std::shared_ptr<ob::Align> align = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
                ob::PointCloudFilter pointCloudFilter;
                pointCloudFilter.setCreatePointFormat(OB_FORMAT_POINT);
                pointCloudFilter.setCameraParam(pipeline->getCameraParam());
                float depthValueScale;

                // Depth only thread
                while (running.load()) {
                    // Waiting for a non-null frameset
                    auto fs = pipeline->waitForFrames(FRAME_WAIT_TIME);
                    if (fs == nullptr) continue;

                    // Frames should be synchronized here
                    if (fs->depthFrame() != nullptr && fs->colorFrame() != nullptr) {
                        // Setting depth scale for point cloud filter
                        depthValueScale = fs->depthFrame()->getValueScale();
                        pointCloudFilter.setPositionDataScaled(depthValueScale);

                        // Alignment processing
                        std::shared_ptr<ob::Frame> depth_aligned = align->process(fs);
                        pointCloudFilter.setCreatePointFormat(OB_FORMAT_POINT);
                        std::shared_ptr<ob::Frame> depth_cloud = pointCloudFilter.process(depth_aligned);
                        if (depth_cloud != nullptr) {
                            // Locking the queue to insert a new depth point cloud, then notifying
                            std::lock_guard<std::mutex> lock(depthQueueMutex);
                            if (depthCloudQueue.size() >= MAX_QUEUE_SIZE) {
                                depthCloudQueue.pop_front();
                            }
                            depthCloudQueue.push_back(convertFrameToPointCloud(depth_cloud, false));
                            depthQueueEmpty.notify_one();
                        } else {
                            throw std::runtime_error("Processed Depth Cloud was NULL");
                        }
                    }
                }

                pipeline->stop();
            };
        }
        
        // Starting the thread
        converter = std::thread(convertFunction, pipeline);
    }

    /**
     * Copies the contents of the frame into a point cloud and returns it
     * @param frame the frame to convert into a point cloud
     * @return the completed point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFrameToPointCloud(const std::shared_ptr<ob::Frame>& ob_frame, bool hasColor) {
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
                    point.y = colorPoints[i].y / 1000.0;
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
                    point.y = points[i].y / 1000.0;
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
    int getCameraWidth(int camera_height) {
        return (camera_height * 16) / 9;
    }
};

#endif // FEMTODRIVER_H