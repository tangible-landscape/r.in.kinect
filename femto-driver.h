#ifndef K4ADRIVER_H
#define K4ADRIVER_H

extern "C" {
    #include <grass/gis.h>
    #include <grass/glocale.h>
}

#include <k4a/k4a.h> // no longer necesary for this header file

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>

#include <string>
#include <iostream>
#include <tuple>
#include <fstream>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>

// New include statements for new Orbbec SDK
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Utils.hpp"
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"

// Mapping from Sensor Type to Enabled Stream
OBStreamType SensorTypeToStreamType(OBSensorType sensorType) {
    switch(sensorType) {
    case OB_SENSOR_COLOR:
        return OB_STREAM_COLOR;
    case OB_SENSOR_DEPTH:
        return OB_STREAM_DEPTH;
    case OB_SENSOR_IR:
        return OB_STREAM_IR;
    case OB_SENSOR_IR_LEFT:
        return OB_STREAM_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
        return OB_STREAM_IR_RIGHT;
    case OB_SENSOR_GYRO:
        return OB_STREAM_GYRO;
    case OB_SENSOR_ACCEL:
        return OB_STREAM_ACCEL;
    default:
        return OB_STREAM_UNKNOWN;
    }
}

class K4ADriver {
public:
    const long unsigned int MAX_DEPTH_QUEUE_SIZE = 4;  // Max number of clouds stored in depthCloudQueue
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
                throw new std::runtime_error("Not implemented D2C");      
            } else {
                throw new std::runtime_error("Not implemented C2D");
            }
        } else {
            std::cout << "Depth Only" << std::endl;
            std::unique_lock<std::mutex> lock(depthQueueMutex);

            // Defining a wait condition for the depth cloud, if we get a depth cloud or thread is killed
            depthCloudEmpty.wait(lock, [this] {
                return !depthCloudQueue.empty() || !running.load();
            });

            if (!depthCloudQueue.empty()) {
                std::cout << "Got depth cloud!" << std::endl;
                cloud = depthCloudQueue.front();
                depthCloudQueue.pop_front();
            } else {
                std::runtime_error("No, cloud found; program terminated early");
            }
        }

        // Attempting to only run once, then deallocate
        shut_down();

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
        depthCloudEmpty.notify_one();  // Waking up the depth cloud
        if (converter.joinable()) converter.join();
    }

private:
    // This is the only queue for now, probably add more queues in the future for different cloud types
    std::mutex depthQueueMutex;  // Mutex for locking the queue of point clouds
    std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> depthCloudQueue;
    std::atomic<bool> running;  // Thread-safe running variable
    std::thread converter;  // The thread for converting all the clouds
    std::condition_variable depthCloudEmpty;

    /**
     * Runs a thread that converts frames from the Femto-Bolt to point clouds and stores them
     * in their respective queues
     */
    void convertClouds() {
        running.store(true);

        converter = std::thread([this]() {
            ob::Pipeline pipeline;
            ob::PointCloudFilter depthPCF;  // Make different filters or ob::Align's later

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
                config->enableStream(colorProfile);
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

            if(depthProfileList->count() > 0) {
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

            // start pipeline with config
            pipeline.start(config);

            // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
            auto cameraParam = pipeline.getCameraParam();
            depthPCF.setCameraParam(cameraParam);

            // Setting Callbacks
            depthPCF.setCallBack([this](std::shared_ptr<ob::Frame> frame) {
                const auto pcl_cloud = convertFrameToPointCloud(frame);
                
                if (pcl_cloud != nullptr) {
                    std::lock_guard<std::mutex> lock(depthQueueMutex);  // Locking the queue for the duration of the context
                    if (depthCloudQueue.size() >= MAX_DEPTH_QUEUE_SIZE) {
                        depthCloudQueue.pop_front();
                    }
                    depthCloudQueue.push_back(pcl_cloud);
                    depthCloudEmpty.notify_one();
                    std::cout << "Processed Depth Frame Callback" << std::endl;
                } else {
                    std::runtime_error("Converted PCL cloud is NULL");
                }
            });

            depthPCF.setCreatePointFormat(OB_FORMAT_POINT);

            // Getting the frames and making the point clouds
            while (running.load()) {
                auto fs = pipeline.waitForFrames(FRAME_WAIT_TIME);
                if (fs == nullptr) continue;
                
                if (fs->depthFrame() != nullptr) {
                    std::cout << "Grabbing a depth frame" << std::endl;
                    auto depthValueScale = fs->depthFrame()->getValueScale();
                    depthPCF.setPositionDataScaled(depthValueScale);
                    depthPCF.pushFrame(fs);  // Gets put in the queue when the callback hits
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFrameToPointCloud(const std::shared_ptr<ob::Frame>& ob_frame) {
        std::cout << "Converting point cloud..." << std::endl; // Debugging

        // If the frame isn't defined or if it isn't a PointsFrame
        if (ob_frame == nullptr || ob_frame->type() != OB_FRAME_POINTS) {
            return nullptr;
        }

        uint32_t cloud_width = 1920; // ob_frame->width();
        uint32_t cloud_height = 1080; // ob_frame->height();

        // Grabbing the points and the data from the Frame
        auto ob_points = ob_frame->as<ob::PointsFrame>();
        std::cout << "Cloud Width and Height: " << cloud_width << " " << cloud_height << std::endl;
        OBPoint *points = (OBPoint *) ob_points->data();
        auto length = ob_points->dataSize() / sizeof(OBPoint);

        // Counting the number of valid points
        static const double epsilon = 1e-6;  // Threshold for distance calculation
        long long int numValidPoints = 0;
        for (long unsigned int i = 0; i < length; i++) {
            if (std::abs(points[i].x) > epsilon && std::abs(points[i].y) > epsilon && std::abs(points[i].z) > epsilon) {
                numValidPoints++;
            }
        }

        std::cout << "Num Valid Points: " << numValidPoints << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(numValidPoints, 1)); // Trying to push into a default initialization
        pcl_cloud->points.resize(numValidPoints);
        pcl_cloud->width = static_cast<uint32_t>(numValidPoints);
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;

        std::cout << "Point Cloud Length: " << length << std::endl;

        bool hasColor = false;
        // Copying the Frame data into the new point cloud
        uint32_t j = 0;  // index for the PCL point cloud
        for (size_t i = 0; i < length; i++) {
            if (std::abs(points[i].x) != 0 && std::abs(points[i].y) != 0 && std::abs(points[i].z) != 0) {
                pcl_cloud->points[j].x = points[i].x / 1000.0;
                pcl_cloud->points[j].y = points[i].y / 1000.0;
                pcl_cloud->points[j].z = points[i].z / 1000.0;
                pcl_cloud->points[j].r = 0; // hasColor ? points[i].r : 0;
                pcl_cloud->points[j].g = 0; // hasColor ? points[i].g : 0;
                pcl_cloud->points[j].b = 0; // hasColor ? points[i].b : 0;
                j++;
            }
        }

        std::cout << "Number of points added: " << j << std::endl;
        if (pcl_cloud->width != j || pcl_cloud->height != 1) {
            throw new std::runtime_error("Invalid point cloud size!");
        }

        std::cout << "Cloud Size:  " << pcl_cloud->size() << std::endl;
        return pcl_cloud;
    }
};

#endif // K4ADRIVER_H
