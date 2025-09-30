#ifndef K4ADRIVER_H
#define K4ADRIVER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>

extern "C" {
    #include <grass/gis.h>
    #include <grass/glocale.h>
    #undef n_  // Fixing a macro collision with PCL
}

#include <k4a/k4a.h> // no longer necesary for this header file

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

// New include statements for new Orbbec SDK
#include "libobsensor/ObSensor.hpp"

class K4ADriver {
public:
    const long unsigned int MAX_QUEUE_SIZE = 2;  // Max number of clouds stored in depthCloudQueue
    const int FRAME_WAIT_TIME = 1000;  // Max number of ms to wait for a frame before refreshing

    K4ADriver() : running(false) {}

    /**
     * Start the cloud conversion thread
     */
    void initialize() {
        // If the thread function is not started, start it
        if (!running.load()) {
            convertClouds();
        }
    }

    /**
     * Grabs the most recent frame from the camera and generates a point cloud
     * @param color if you want to include color in the point cloud
     * @param depth2color if you want to include a depth to color mapping in the point cloud,
     * else include a color2depth mapping
     * @return the point cloud with the specified parameters
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(bool color, bool depth2color) {
        // Start the thread if the user forgot to call init
        if (!running.load()) {
            convertClouds();
        }        

        // Checking cloud options
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        if (color) {
            if (depth2color) {
                std::cout << "Depth 2 Color" << std::endl;

                std::unique_lock<std::mutex> lock(d2cQueueMutex);
                d2cQueueEmpty.wait(lock, [this]{
                    return !d2cCloudQueue.empty() || !running.load();
                });

                if (!d2cCloudQueue.empty()) {
                    cloud = d2cCloudQueue.front();
                    d2cCloudQueue.pop_front();
                } else {
                    throw std::runtime_error("No D2C cloud found, program terminated early");
                }
            } else {
                std::cout << "Color 2 Depth" << std::endl;

                std::unique_lock<std::mutex> lock(c2dQueueMutex);
                c2dQueueEmpty.wait(lock, [this]{
                    return !c2dCloudQueue.empty() || !running.load();
                });

                if (!c2dCloudQueue.empty()) {
                    cloud = c2dCloudQueue.front();
                    c2dCloudQueue.pop_front();
                } else {
                    throw std::runtime_error("No C2D cloud found, program terminated early");
                }
            }
        } else {
            std::cout << "Depth Only" << std::endl;
            std::unique_lock<std::mutex> lock(depthQueueMutex);

            // Defining a wait condition for the depth cloud, if we get a depth cloud or thread is killed
            depthQueueEmpty.wait(lock, [this] {
                return !depthCloudQueue.empty() || !running.load();
            });

            if (!depthCloudQueue.empty()) {
                std::cout << "Got depth cloud!" << std::endl;
                cloud = depthCloudQueue.front();
                depthCloudQueue.pop_front();
            } else {
                std::runtime_error("No depth cloud found; program terminated early");
            }
        }

        return cloud;
    }

    void release() {
        throw std::runtime_error("Unimplemented Method Exception");
    }

    /**
     * Kills the converter thread, and shuts down the pipeline
     */
    void shut_down() {
        running.store(false);
        depthQueueEmpty.notify_one();  // Waking up the depth cloud
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

    /**
     * Runs a thread that converts frames from the Femto-Bolt to point clouds and stores them
     * in their respective queues
     */
    void convertClouds() {
        running.store(true);

        converter = std::thread([this]() {
            ob::Pipeline pipeline;
            // Using a single filter, just change the output mode to Depth or Depth + RGB
            ob::PointCloudFilter pointCloudFilter;

            std::shared_ptr<ob::Align> d2cAlign = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            std::shared_ptr<ob::Align> c2dAlign = std::make_shared<ob::Align>(OB_STREAM_DEPTH);

            // Pipeline and PCF configuration logic
            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

            // Configure which streams to enable or disable for the Pipeline by creating a Config
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

            // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds

            std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
            try {
                // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
                auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
                if(colorProfiles) {
                    auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
                    colorProfile = profile->as<ob::VideoStreamProfile>();
                }
                config->enableVideoStream(OB_SENSOR_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY,
                                            OB_FORMAT_BGR);
            }
            catch(ob::Error &e) {
                config->setAlignMode(ALIGN_DISABLE);
                std::cerr << "Current device is not support color sensor!" << std::endl;
            }

            // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
            std::shared_ptr<ob::StreamProfileList> depthProfileList;
            OBAlignMode                            alignMode = ALIGN_DISABLE;
            if(colorProfile) {
                // Try find supported depth to color align hardware mode profile
                depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
                if(depthProfileList->count() > 0) {
                    alignMode = ALIGN_D2C_HW_MODE;
                }
                else {
                    // Try find supported depth to color align software mode profile
                    depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                    if(depthProfileList->count() > 0) {
                        // I'm pretty sure that only this mode is supported on the Femto-Bolt
                        alignMode = ALIGN_D2C_SW_MODE;
                    }
                }

                try {
                    // Enable frame synchronization
                    pipeline.enableFrameSync();
                    std::cout << "Enabled frame sync!" << std::endl;
                }
                catch(ob::Error &e) {
                    std::cerr << "Current device is not support frame sync!" << std::endl;
                }
            }
            else {
                depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
            }

            if (depthProfileList->count() > 0) {
                std::shared_ptr<ob::StreamProfile> depthProfile;
                try {
                    // Select the profile with the same frame rate as color.
                    if(colorProfile) {
                        depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
                    }
                }
                catch(...) {
                    depthProfile = nullptr;
                }

                if(!depthProfile) {
                    // If no matching profile is found, select the default profile.
                    depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
                }
                config->enableStream(depthProfile);
            }
            config->setAlignMode(alignMode);

            // Essential for frame alignment
            config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

            // start pipeline with config
            pipeline.start(config);

            // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
            auto cameraParam = pipeline.getCameraParam();
            pointCloudFilter.setCameraParam(cameraParam);

            // Looking into the recommended filters for the point cloud
            /*
            auto device = pipeline.getDevice();
            auto sensor = device->getSensor(OB_SENSOR_DEPTH);
            auto filterList = sensor->createRecommendedFilters();
            auto special_filter = filterList[2];
            special_filter->enable(true);
            std::cout << special_filter->getName() << std::endl;

            Unsupported:
            0: Decimation filter
            1: Spatial Filter
            2: Temporal Filter
            */

            // Getting the frames and making the point clouds
            float depthValueScale;
            while (running.load()) {
                auto fs = pipeline.waitForFrames(FRAME_WAIT_TIME);
                if (fs == nullptr) continue;

                // Enforcing frame synchronization, so frames should be together
                // Trying synchronous methods first
                if (fs->depthFrame() != nullptr && fs->colorFrame() != nullptr) {
                    depthValueScale = fs->depthFrame()->getValueScale();
                    pointCloudFilter.setPositionDataScaled(depthValueScale);

                    // Alignment processing
                    std::shared_ptr<ob::Frame> depth_aligned = d2cAlign->process(fs);
                    std::shared_ptr<ob::Frame> c2d_aligned = c2dAlign->process(fs);
                    std::shared_ptr<ob::Frame> d2c_aligned = d2cAlign->process(fs);
                                        
                    // Depth
                    pointCloudFilter.setCreatePointFormat(OB_FORMAT_POINT);
                    std::shared_ptr<ob::Frame> depth_cloud = pointCloudFilter.process(depth_aligned);
                    if (depth_cloud != nullptr) {
                        std::lock_guard<std::mutex> lock(depthQueueMutex);
                        if (depthCloudQueue.size() >= MAX_QUEUE_SIZE) {
                            depthCloudQueue.pop_front();
                        }
                        depthCloudQueue.push_back(convertFrameToPointCloud(depth_cloud, false));
                        depthQueueEmpty.notify_one();
                        std::cout << "Processed Depth Cloud" << std::endl;
                    } else {
                        throw std::runtime_error("Processed Depth Cloud was NULL");
                    }

                    // C2D - Currently unsupported by the align tool
                    pointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                    std::shared_ptr<ob::Frame> c2d_cloud = pointCloudFilter.process(c2d_aligned);
                    if (c2d_cloud != nullptr) {
                        std::lock_guard<std::mutex> lock(c2dQueueMutex);
                        if (c2dCloudQueue.size() >= MAX_QUEUE_SIZE) {
                            c2dCloudQueue.pop_front();
                        }
                        c2dCloudQueue.push_back(convertFrameToPointCloud(c2d_cloud, true));
                        c2dQueueEmpty.notify_one();
                        std::cout << "Processed Color to Depth Cloud" << std::endl;
                    } else {
                        throw std::runtime_error("Processed C2D Cloud was NULL");
                    }

                    // D2C
                    std::shared_ptr<ob::Frame> d2c_cloud = pointCloudFilter.process(d2c_aligned);
                    if (d2c_cloud != nullptr) {
                        std::lock_guard<std::mutex> lock(d2cQueueMutex);
                        if (d2cCloudQueue.size() >= MAX_QUEUE_SIZE) {
                            d2cCloudQueue.pop_front();
                        }
                        d2cCloudQueue.push_back(convertFrameToPointCloud(d2c_cloud, true));
                        d2cQueueEmpty.notify_one();
                        std::cout << "Processed Depth to Color Cloud" << std::endl;
                    } else {
                        throw std::runtime_error("Processed D2C Cloud was NULL");
                    }
                }
            }

            // Shutting down the pipeline outside of the thread
            pipeline.stop();
        });
    }

    /**
     * Copies the contents of the frame into a point cloud and returns it
     * @param frame the frame to convert into a point cloud
     * @return the completed point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFrameToPointCloud(const std::shared_ptr<ob::Frame>& ob_frame, bool hasColor) {
        std::cout << "Converting point cloud..." << std::endl; // Debugging

        // If the frame isn't defined or if it isn't a PointsFrame
        if (ob_frame == nullptr || ob_frame->type() != OB_FRAME_POINTS) {
            return nullptr;
        }

        // Grabbing the points and the data from the Frame
        auto ob_points = ob_frame->as<ob::PointsFrame>();
        OBColorPoint *colorPoints;
        OBPoint *points;
        int length = 0;
        if (hasColor) {
            colorPoints = (OBColorPoint *) ob_points->data();
            length = ob_points->dataSize() / sizeof(OBColorPoint);
        } else {
            points = (OBPoint *) ob_points->data();
            length = ob_points->dataSize() / sizeof(OBPoint);
        }

        // Initializing the point cloud
        static const double epsilon = 1e-6;  // Threshold for distance calculation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        std::cout << "Point Cloud Length: " << length << std::endl;

        // Copying the Frame data into the new point cloud
        uint32_t j = 0;  // index for the PCL point cloud
        if (hasColor) {
            for (size_t i = 0; i < length; i++) {
                if (std::abs(colorPoints[i].x) > epsilon && std::abs(colorPoints[i].y) > epsilon && std::abs(colorPoints[i].z) > epsilon) {
                    // Swapping red and blue to account for some swap somewhere
                    pcl_cloud->points.push_back(pcl::PointXYZRGB(
                        static_cast<float>(colorPoints[i].x / 1000.0),
                        static_cast<float>(-colorPoints[i].y / 1000.0),
                        static_cast<float>(colorPoints[i].z / 1000.0),
                        static_cast<std::uint8_t>(colorPoints[i].b),
                        static_cast<std::uint8_t>(colorPoints[i].g),
                        static_cast<std::uint8_t>(colorPoints[i].r)
                    ));
                }
            }
        } else {
            for (size_t i = 0; i < length; i++) {
                if (std::abs(points[i].x) > epsilon && std::abs(points[i].y) > epsilon && std::abs(points[i].z) > epsilon) {
                    pcl_cloud->points.push_back(pcl::PointXYZRGB(
                        static_cast<float>(points[i].x / 1000.0),
                        static_cast<float>(-points[i].y / 1000.0),
                        static_cast<float>(points[i].z / 1000.0),
                        static_cast<std::uint8_t>(0),
                        static_cast<std::uint8_t>(0),
                        static_cast<std::uint8_t>(0)
                    ));
                } 
            }
        }

        // Resizing the point cloud, if necessary
        if (pcl_cloud->points.size() != pcl_cloud->height * pcl_cloud->width) {
            pcl_cloud->height = static_cast<std::uint32_t>(1);
            pcl_cloud->width = static_cast<std::uint32_t>(pcl_cloud->points.size());
        }
        pcl_cloud->is_dense = true;

        std::cout << "Cloud Size:  " << pcl_cloud->size() << std::endl;
        return pcl_cloud;
    }
};

#endif // K4ADRIVER_H
