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

// New include statements for new Orbbec SDK
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Utils.hpp"
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"

// Global Variables
// This is very long for testing purposes, just so I can make sure that it's not the issue
#define TIMEOUT_DURATION 5000  // The timeout duration to wait for frames, in milliseconds

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
    // Maybe global variables instead of class members so they never go out of scope
    ob::Pipeline pipeline;
    ob::PointCloudFilter depthPCF;
    
    K4ADriver() {}

    /**
     * Initialize the driver using the parameters 
     * scanned from the Orbbec Camera's connection
     */
    void initialize() {
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
        // colorPCF.setCameraParam(cameraParam);

        // Setting Callbacks
        depthPCF.setCallBack([this](std::shared_ptr<ob::Frame> frame) {
            this->depthCloud = convertFrameToPointCloud(frame);
            std::cout << "Processed Depth Frame Callback" << std::endl;
        });
        /*
        colorPCF.setCallBack([this](std::shared_ptr<ob::Frame> frame) {
            this->colorFrame = frame;
            std::cout << "Processed Color Frame Callback" << std::endl;
        });
        */

        // Setting the point formats
        // colorPCF.setCreatePointFormat(OB_FORMAT_RGB_POINT);
        depthPCF.setCreatePointFormat(OB_FORMAT_POINT);

        /*
        // Setting the logger severity to a warning default
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

        // Configure which streams to enable or disable for the Pipeline by creating a Config
        config = std::make_shared<ob::Config>();

        // Set-up for stream configuration
        auto device = pipeline.getDevice();
        auto sensorList = device->getSensorList();
        for (int i = 0; i < sensorList->count(); i++) {
            auto sensorType = sensorList->type(i);
            if (sensorType == OB_SENSOR_GYRO || sensorType == OB_SENSOR_ACCEL) {
               continue;
            }
            auto streamType = SensorTypeToStreamType(sensorType);
            config->enableVideoStream(streamType);
        }
        
        // Setting Alignment Modes
        depth2ColorAlign = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        color2DepthAlign = std::make_shared<ob::Align>(OB_STREAM_DEPTH);

        // Setting callbacks to their respective methods
        depth2ColorAlign->setCallBack([this](std::shared_ptr<ob::Frame> frame) {
            // std::unique_lock<std::mutex> lock(frameMutex);
            this->colorFrame = frame;});
        color2DepthAlign->setCallBack([this](std::shared_ptr<ob::Frame> frame) {
            // std::unique_lock<std::mutex> lock(frameMutex);
            this->depthFrame = frame;
        });

        // Callback to handle frame updates
        pipeline.start(config, [&](std::shared_ptr<ob::FrameSet> frameset) {
            auto count = frameset->frameCount();
            for (int i = 0; i < count; i++) {
                auto frame = frameset->getFrame(i);
                std::unique_lock<std::mutex> lk(frameMutex);
                if (frame->type() == OB_FRAME_DEPTH) {
                    depth2ColorAlign->pushFrame(frame);
                } else {
                    color2DepthAlign->pushFrame(frame);
                }
            }
        });
        */

        /*

        // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
        try {
            // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
            auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
            if (colorProfiles) {
                auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
                colorProfile = profile->as<ob::VideoStreamProfile>();
            }
            config->enableStream(colorProfile);
        } catch(ob::Error &e) {
            config->setAlignMode(ALIGN_DISABLE);
            std::cerr << "Current device doesn't support color sensor!" << std::endl;
        }

        // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
        std::shared_ptr<ob::StreamProfileList> depthProfileList;
        OBAlignMode alignMode = ALIGN_DISABLE;
        if (colorProfile) {
            // Try find supported depth to color align hardware mode profile
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
            } else {
                // Try find supported depth to color align software mode profile
                depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if (depthProfileList->count() > 0) {
                    alignMode = ALIGN_D2C_SW_MODE;
                }
            }

            try {
                // Enable frame synchronization
                pipeline.enableFrameSync();
            } catch(ob::Error &e) {
                std::cerr << "Current device doesn't support frame sync!" << std::endl;
            }
        } else {
            depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
        }

        if (depthProfileList->count() > 0) {
            std::shared_ptr<ob::StreamProfile> depthProfile;
            try {
                // Select the profile with the same frame rate as color.
                if (colorProfile) {
                    depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
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
            */

        // Aggregating frames
        // config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        // config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);
        // config->setAlignMode(alignMode);

        // Starting the pipeline with the constructed config
        // pipeline.start(config);

        // Initializing the point cloud with the parameters from the Camera
        // pointCloud.setCameraParam(pipeline.getCameraParam());
    }

    /**
     * Grabs the most recent frame from the camera and generates a point cloud
     * @param color if you want to include color in the point cloud
     * @param depth2color if you want to include a depth to color mapping in the point cloud,
     * else include a color2depth mapping
     * @return the point cloud with the specified parameters
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(bool color, bool depth2color) {
        // Waiting for both frames to sync up
        std::cout << "get_cloud called!" << std::endl;
        // These will store the processed frames after they are gathered
        const float DEFAULT = -1.0f;
        float depthValueScale = DEFAULT;
        int index = 0;
        // this->haveDepth = false; Only run this once to configure the cloud
        while (!this->haveDepth) {
            auto fs = pipeline.waitForFrames(1000);
            if (fs == nullptr) continue;
            
            std::cout << "Loop Index: " << index++ << std::endl;
            /*
            if (!haveColor && fs->colorFrame() != nullptr && depthValueScale != DEFAULT) {
                std::cout << "Grabbing a color frame" << std::endl;
                colorPCF.setPositionDataScaled(depthValueScale);
                this->haveColor = true;
                colorPCF.pushFrame(fs);  // Gets assigned when the callback hits
            }
            */
            
            if (!haveDepth && fs->depthFrame() != nullptr) {
                std::cout << "Grabbing a depth frame" << std::endl;
                depthValueScale = fs->depthFrame()->getValueScale();
                depthPCF.setPositionDataScaled(depthValueScale);
                this->haveDepth = true;
                depthPCF.pushFrame(fs);  // Gets assigned when the callback hits
            }
        }

        while (this->depthCloud == nullptr) {
            // Busy waiting for the async to process, this is bad practice but temporary
        }
        
        // These are different frames...
        /*
        if (colorFrame) {
            std::cout << "Color Frame: " << colorFrame->width() << "x" << colorFrame->height() << " " << colorFrame->timeStampUs() << " us" << std::endl;
        }

        if(depthFrame) {
            std::cout << "Depth Frame: " << depthFrame->width() << "x" << depthFrame->height() << " " << depthFrame->timeStampUs() << " us" << std::endl;
        }
        */

        /*  Don't need because it's in a callback
        // Getting the frames from the camera
        frameset = pipeline.waitForFrames(TIMEOUT_DURATION);

        // For debugging purposes, remove later
        if (frameset == nullptr) std::cout << "Frame Null" << std::endl;

        // Updating framesets if they're non-null
        if (frameset->depthFrame() != nullptr) {
            std::cout << "Depth to Color Processing" << std::endl;
            this->colorFrame = depth2ColorAlign->process(frameset);
        }
        if (frameset->colorFrame() != nullptr) {
            std::cout << "Color to Depth Processing" << std::endl;
            this->depthFrame = color2DepthAlign->process(frameset);
        }
            */

        /*
        if (frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
            // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
            // millimeter)
            // I don't know if this is needed
            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);
            try {
                pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                std::cout << "Saved frame successfully!" << std::endl;
            } catch (std::exception &e) {
                std::cout << "Get point cloud failed" << std::endl;
            }
        } else {
            std::cout << "Get color frame or depth frame failed!" << std::endl;    
        }
        */

        // TODO: Update this so that we can still get some information if either the depth or color fails

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (color) {
            if (depth2color) {
                std::cout << "Depth 2 Color" << std::endl; 
                throw new std::runtime_error("Not implemented D2C");      
                // cloud = prepare_cloud_RGBD_D2C(depthFrame);
            } else {
                std::cout << "Color 2 Depth" << std::endl;    
                throw new std::runtime_error("Not implemented D2C");      
                // cloud = prepare_cloud_RGBD_C2D(colorFrame);
            }
        } else {
            std::cout << "Depth Only" << std::endl;    
            cloud = this->depthCloud;
        }

        std::cout << "Final Cloud Size: " << cloud->size() << std::endl;

        /*
        This was tested and works successfully
        // Trying to make a KDTree, since that is what's going wrong
        pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);
        std::cout << "Created KDTree" << std::endl;
        */
        return cloud;
    }

    void release() {
        throw std::runtime_error("Unimplemented Method Exception");
    }

    void shut_down() {
        // Stopping the pipeline
        pipeline.stop();
    }

private:
    std::mutex frameMutex;  // Mutex for locking the frames
    // Two point cloud filters for depth and color
    // ob::PointCloudFilter colorPCF;
    // Processed frames for the point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud;
    // std::shared_ptr<ob::Frame> depthFrame;
    // std::shared_ptr<ob::Frame> colorFrame;
    // Flags to indicate if the frames are processed, used for more immediate feedback than the callback
    bool haveColor = false;
    bool haveDepth = false;

    /**
     * Prepares a point cloud with depth only
     * @return a PCL point cloud with depth information only
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepare_cloud_D(const std::shared_ptr<ob::Frame>& frame) {
        // Grabbing the depth image from the frameset
        if (frame == nullptr) throw std::runtime_error("Failed to get depth image from capture");
        
        // Creating a new point cloud object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = convertFrameToPointCloud(frame);
        if (pcl_cloud == nullptr) std::runtime_error("Failed to convert depth frame to point cloud");

        return pcl_cloud;
    }

    /**
     * Returning a color-to-depth mapped cloud from a depth frame
     * @param depthFrame the depth frame to convert to a point cloud
     * @return the depth-to-color mapped PCL point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepare_cloud_RGBD_C2D(const std::shared_ptr<ob::Frame>& frame) {
        if (frame == nullptr) {
            throw std::runtime_error("No depth frame in cache yet");
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = convertFrameToPointCloud(frame);
        if (pcl_cloud == nullptr) std::runtime_error("Failed to convert depth frame to point cloud");

        return pcl_cloud;
    }

    /**
     * Returning the depth-to-color mapped cloud from a color frame
     * This should be the default behavior of the frameset, so we shouldn't have the change
     * the pipeline configuration to get the desired behavior
     * @return the depth-to-color mapped point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prepare_cloud_RGBD_D2C(const std::shared_ptr<ob::Frame>& frame) {
        if (frame == nullptr) throw std::runtime_error("No color frame in cache yet");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = convertFrameToPointCloud(frame);
        if (pcl_cloud == nullptr) std::runtime_error("Failed to convert color frame to point cloud");

        return pcl_cloud;
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

        // Defining a new PCL point cloud with the right size
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(numValidPoints, 1));
        // Making a dense, unorganized point cloud with the right size
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(numValidPoints, 1)); // Trying to push into a default initialization
        pcl_cloud->points.resize(numValidPoints);
        pcl_cloud->width = static_cast<uint32_t>(numValidPoints);
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;
        // pcl_cloud->is_dense = true; // We're filtering out the invalid points
        //pcl_cloud->points.resize(numValidPoints);

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
        /*
        if (j != cloud_width * cloud_height) pcl_cloud->points.resize(j);
        pcl_cloud->height = 1;
        pcl_cloud->width = j;
        pcl_cloud->is_dense = true;
        */
        std::cout << "Cloud Size:  " << pcl_cloud->size() << std::endl;

        /*
        // Attempting to visualize the point cloud 
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(pcl_cloud);
        while (!viewer.wasStopped()) {}
        */

        return pcl_cloud;
    }
};

#endif // K4ADRIVER_H
