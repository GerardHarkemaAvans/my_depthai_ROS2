/**
	stereo_inertial_roboflow_publisher.cpp
	Purpose: ROS Implementation for OAK-D camera
	@author Gerard Harkema
	@version 0.9 2023/01/05
    License: CC BY-NC-SA
*/

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <cassert> // assert
#include <bits/stdc++.h>

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"


#include "jsoncpp/json/json.h"

#include "processing.h"
#include "pipe.h"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_inertial_node");

    //ros::NodeHandle pnh("~");


    std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath, nnConfigPath;
    std::string monoResolution = "720p", rgbResolution = "1080p";
    int badParams = 0, stereo_fps, confidence, LRchecktresh, imuModeParam, expTime, sensIso;
    int rgbScaleNumerator, rgbScaleDinominator, previewWidth, previewHeight;
    bool lrcheck, extended, subpixel, rectify,  manualExposure;
    bool enableNeuralNetworkDetection, enableDotProjector, enableFloodLight;
    bool usb2Mode, poeMode;
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectormA, floodLightmA;
    float confidenceThreshold = 0.5;
    float overlapThreshold = 0.5;
    int boxNeighbors;
    std::string nnName(BLOB_NAME), nnConfig(BLOB_NAME);  // Set your blob name for the model here

    node->declare_parameter("mxId", mxId);
    node->declare_parameter("usb2Mode", usb2Mode);
    node->declare_parameter("poeMode", poeMode);

    node->declare_parameter("tf_prefix", tfPrefix);
    node->declare_parameter("mode", mode);
    node->declare_parameter("imuMode", imuModeParam);

    node->declare_parameter("lrcheck", lrcheck);
    node->declare_parameter("extended", extended);
    node->declare_parameter("subpixel", subpixel);
    node->declare_parameter("rectify", rectify);

    node->declare_parameter("stereo_fps", stereo_fps);
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("rgbResolution", rgbResolution);
    node->declare_parameter("manualExposure", manualExposure);
    node->declare_parameter("expTime", expTime);
    node->declare_parameter("sensIso", sensIso);

    node->declare_parameter("rgbScaleNumerator", rgbScaleNumerator);
    node->declare_parameter("rgbScaleDinominator", rgbScaleDinominator);
    node->declare_parameter("previewWidth", previewWidth);
    node->declare_parameter("previewHeight", previewHeight);

    node->declare_parameter("angularVelCovariance", angularVelCovariance);
    node->declare_parameter("linearAccelCovariance", linearAccelCovariance);
    node->declare_parameter("enableNeuralNetworkDetection", enableNeuralNetworkDetection);


    // Applies only to PRO model
    node->declare_parameter("enableDotProjector", enableDotProjector);
    node->declare_parameter("enableFloodLight", enableFloodLight);
    node->declare_parameter("dotProjectormA", dotProjectormA);
    node->declare_parameter("floodLightmA", floodLightmA);

    if(enableNeuralNetworkDetection){
        node->declare_parameter("resourceBaseFolder", resourceBaseFolder);
        node->declare_parameter("confidenceThreshold", confidenceThreshold);
        node->declare_parameter("overlapThreshold", overlapThreshold);
        node->declare_parameter("boxNeighbors", boxNeighbors);
        node->declare_parameter("nnName", nnName);
        node->declare_parameter("nnConfig", nnConfig);
    }

    if(badParams > 0) {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }


    if(enableNeuralNetworkDetection){
        if(resourceBaseFolder.empty()) {
            throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
        }
        nnPath = resourceBaseFolder + "/" + nnName;
        nnConfigPath = resourceBaseFolder + "/" + nnConfig;
        std::cout << " NeuralNetworkDetection enabled: nnPath: " << nnPath << std::endl;
        std::cout << " NeuralNetworkDetection enabled: nnConfigPath: " << nnConfigPath << std::endl;
    }
    else
        std::cout << " NeuralNetworkDetection disabled" << std::endl;

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);


    int nn_width, nn_height;
    std::vector<std::string> class_names;
    bool yolo_nn_network = false;
    if(enableNeuralNetworkDetection){
        std::ifstream file(nnConfigPath);
        // json reader
        Json::Reader reader;
        // this will contain complete JSON data
        Json::Value completeJsonData;
        // reader reads the data and stores it in completeJsonData
        reader.parse(file, completeJsonData);
        Json::Value classes;

        /* select nn network type */
        if(!completeJsonData["environment"]["RESOLUTION"].isNull()){
            /* Created by Roboflow */
            yolo_nn_network = false;
            nn_width = std::stoi(completeJsonData["environment"]["RESOLUTION"].asString());
            nn_height = nn_width;

            classes = completeJsonData["class_names"];
            //std::cout << "not a yolo network" << std::endl;
        }
        else{
            /* Created by pyTorch */
            yolo_nn_network = true;
            classes = completeJsonData["mappings"]["labels"];
            //std::cout << "!!! yolo network !!!!" << std::endl;
            const Json::Value nn_width_json = completeJsonData["nn_config"]["input_size"].asString();
            nn_width = 416; // nog nie juist!!!!
            nn_height = nn_width;
        }



        for(int i = 0; i < classes.size(); i++){
            class_names.push_back(classes[i].asString());
            //std::cout << classes[i].asString() << std::endl;
        }
    }


    dai::Pipeline pipeline;
    int image_width, image_height;
    bool isDeviceFound = false;
    std::tie(pipeline, image_width, image_height) = createPipeline(
                                                        enableNeuralNetworkDetection,
                                                        lrcheck,
                                                        extended,
                                                        subpixel,
                                                        rectify,
                                                        stereo_fps,
                                                        confidence,
                                                        LRchecktresh,
                                                        monoResolution,
                                                        rgbResolution,
                                                        rgbScaleNumerator,
                                                        rgbScaleDinominator,
                                                        previewWidth,
                                                        previewHeight,
                                                        nnPath,
                                                        nnConfigPath,
                                                        confidenceThreshold,
                                                        yolo_nn_network);

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    for(auto deviceInfo : availableDevices) {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if(deviceInfo.getMxId() == mxId) {
            if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                if(poeMode) {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                } else {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
                }
                break;
            } else if(deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" is already booted on different process.  \"");
            }
        } else if(mxId == "x") {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }

    if(!isDeviceFound) {
        throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }

    if(!poeMode) {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }

    // Apply camera controls
    auto controlQueue = device->getInputQueue("control");

    //Set manual exposure
    if(manualExposure){
        dai::CameraControl ctrl;
        ctrl.setManualExposure(expTime, sensIso);
        controlQueue->send(ctrl);
    }


    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    stereoQueue = device->getOutputQueue("depth", 30, false);

    auto imuQueue = device->getOutputQueue("imu", 30, false);

    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE" && depth_aligned == false) {
        width = 640;
        height = 480;
    }
    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) {
            device->setIrLaserDotProjectorBrightness(dotProjectormA);
        }

        if(enableFloodLight) {
            device->setIrFloodLightBrightness(floodLightmA);
        }
    }

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        converter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        rightconverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");



    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    if(enableRosBaseTimeUpdate) {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();


    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);

    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, image_width, image_height);


    auto depthconverter = rgbConverter;
    auto depthCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, image_width, image_height);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        pnh,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &depthconverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                  std::placeholders::_1,
                  std::placeholders::_2),
        30,
        depthCameraInfo,
        "stereo");
    depthPublish.addPublisherCallback();

    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, image_width, image_height);
    auto imgQueue = device->getOutputQueue("rgb", 30, false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
        imgQueue,
        pnh,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");
    rgbPublish.addPublisherCallback();

    //ros::spin();
    //return 0;

    if(enableNeuralNetworkDetection) {
        auto previewQueue = device->getOutputQueue("preview", 30, false);
        auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, previewWidth, previewHeight);

        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
            previewQueue,
            pnh,
            std::string("color/preview/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            previewCameraInfo,
            "color/preview");
        previewPublish.addPublisherCallback();

        std::unique_ptr<std::thread> detection_task_ptr;

        if(yolo_nn_network){

            auto detectionQueue = device->getOutputQueue("detections", 30, false);

            //dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
            dai::rosBridge::SpatialDetectionConverterEx detConverter(tfPrefix + "_rgb_camera_optical_frame", image_width, image_height, false);
            dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
                detectionQueue,
                pnh,
                std::string("color/detections"),
                std::bind(&dai::rosBridge::SpatialDetectionConverterEx::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                30);

            detectionPublish.addPublisherCallback();

            rclcpp::spin(node);
            return 0;

        }
        else{
            std::thread detection_task(DetectionTask,
                                        device,
                                        "detections",
                                        "spatialData",
                                        "spatialCalcConfig",
                                        pnh.getNamespace() + "/color/detections",
                                        image_width,  image_height,
                                        nn_width, nn_height,
                                        class_names,
                                        confidenceThreshold,
                                        overlapThreshold,
                                        boxNeighbors);
            rclcpp::spin(node);
            AbortDetectionTask();
            detection_task.join();
        }
    }
    else{
        rclcpp::spin(node);
        return 0;
    }
    return 0;
}
