
#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

#include "jsoncpp/json/json.h"

std::tuple<dai::Pipeline, int, int> createPipeline(bool lrcheck, bool extended, bool syncNN, bool subpixel, std::string nnPath, std::string nnConfigPath, int confidence, int LRchecktresh, std::string resolution) {

#if 0
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto monoLeftCam = pipeline.create<dai::node::MonoCamera>();
    auto monoRightCam = pipeline.create<dai::node::MonoCamera>();
    auto stereoDepth = pipeline.create<dai::node::StereoDepth>();
    //auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();

    // create xlink connections
    auto xoutColor = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    //auto xoutNN = pipeline.create<dai::node::XLinkOut>();


    //spatial_camera = true;

    std::cout << "Spatial_camera == true" <<std::endl;

    //xoutNN->setStreamName("detections");
    xoutDepth->setStreamName("depth");
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutColor->setStreamName("color");


    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    monoLeftCam->setResolution(monoResolution);
    monoLeftCam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRightCam->setResolution(monoResolution);
    monoRightCam->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    /// setting node configs
    stereoDepth->initialConfig.setConfidenceThreshold(confidence);
    stereoDepth->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereoDepth->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereoDepth->setSubpixel(subpixel);
    stereoDepth->setDepthAlign(dai::CameraBoardSocket::CAM_A);
#endif
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto monoLeftCam = pipeline.create<dai::node::MonoCamera>();
    auto monoRightCam = pipeline.create<dai::node::MonoCamera>();
    auto stereoDepth = pipeline.create<dai::node::StereoDepth>();

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    auto xoutColor = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutColor->setStreamName("color");
    xoutDepth->setStreamName("depth");

    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeftCam->setResolution(monoResolution);
    monoLeftCam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRightCam->setResolution(monoResolution);
    monoRightCam->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // StereoDepth
    stereoDepth->initialConfig.setConfidenceThreshold(confidence);
    stereoDepth->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereoDepth->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereoDepth->setLeftRightCheck(lrcheck);
    stereoDepth->setExtendedDisparity(extended);
    stereoDepth->setSubpixel(subpixel);
    stereoDepth->setDepthAlign(dai::CameraBoardSocket::CAM_A); //added by gerard


//stereoDepth->setExtendedDisparity(extended);;
//stereoDepth->setDepthAlign(dai::CameraBoardSocket::RGB);

#if 0
    {
        std::ifstream file(nnConfigPath);
        // json reader
        Json::Reader reader;
        // this will contain complete JSON data
        Json::Value completeJsonData;
        // reader reads the data and stores it in completeJsonData
        reader.parse(file, completeJsonData);
        //std::cout << completeJsonData << std::endl;
        //std::cout << completeJsonData["nn_config"]["NN_specific_metadata"]["confidence_threshold"].asString() << std::endl;

        spatialDetectionNetwork->setBlobPath(nnPath);
        spatialDetectionNetwork->setConfidenceThreshold(0.5f);//confidenceThreshold);//std::stof(completeJsonData["nn_config"]["NN_specific_metadata"]["confidence_threshold"].asString()));
        spatialDetectionNetwork->input.setBlocking(false);
        spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
        spatialDetectionNetwork->setDepthLowerThreshold(100);
        spatialDetectionNetwork->setDepthUpperThreshold(5000);

        // yolo specific parameters
        spatialDetectionNetwork->setNumClasses(std::stoi(completeJsonData["nn_config"]["NN_specific_metadata"]["classes"].asString()));
        spatialDetectionNetwork->setCoordinateSize(std::stoi(completeJsonData["nn_config"]["NN_specific_metadata"]["coordinates"].asString()));


        /* extract anchors */
        std::vector<float> anchors;
        Json::Value anchors_json = completeJsonData["nn_config"]["NN_specific_metadata"]["anchors"];

        for(int i = 0; i < anchors_json.size(); i++){
            anchors.push_back(std::stof(anchors_json[i].asString()));
        }
        spatialDetectionNetwork->setAnchors(anchors);

        /* extract anchor masks */
        std::map<std::string, std::vector<int>> anchorMasks;
        Json::Value anchors_mask_json = completeJsonData["nn_config"]["NN_specific_metadata"]["anchor_masks"];

        for (auto const& id : anchors_mask_json.getMemberNames()) {
            Json::Value anchors_mask_members_json = anchors_mask_json[id];
            std::vector<int> mask_values;
            for(int i = 0; i < anchors_mask_members_json.size(); i++){
                mask_values.push_back(std::stoi(anchors_mask_members_json[i].asString()));
            }

            anchorMasks[id] = mask_values;
            mask_values.clear();
        }
        spatialDetectionNetwork->setAnchorMasks(anchorMasks);

        spatialDetectionNetwork->setIouThreshold(std::stof(completeJsonData["nn_config"]["NN_specific_metadata"]["iou_threshold"].asString()));


        spatialDetectionNetwork->setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::MIN);
    }
#endif
#if 0
    // Link plugins CAM -> STEREO -> XLINK
    monoLeftCam->out.link(stereoDepth->left);
    monoRightCam->out.link(stereoDepth->right);

    stereoDepth->rectifiedLeft.link(xoutLeft->input);
    stereoDepth->rectifiedRight.link(xoutRight->input);
    colorCam->preview.link(xoutColor->input);
    stereoDepth->depth.link(xoutDepth->input);


    syncNN = true; // added by gerard

    // Link plugins CAM -> NN -> XLINK
    //colorCam->preview.link(spatialDetectionNetwork->input);
#if 0
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xoutColor->input);
    else
        colorCam->preview.link(xoutColor->input);
#endif

    colorCam->preview.link(xoutColor->input);
    //spatialDetectionNetwork->out.link(xoutNN->input);

    //stereoDepth->depth.link(spatialDetectionNetwork->inputDepth);
    stereoDepth->depth.link(xoutDepth->input);
    //spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
#endif
    // Link plugins CAM -> STEREO -> XLINK
    monoLeftCam->out.link(stereoDepth->left);
    monoRightCam->out.link(stereoDepth->right);

    stereoDepth->rectifiedLeft.link(xoutLeft->input);
    stereoDepth->rectifiedRight.link(xoutRight->input);
    colorCam->preview.link(xoutColor->input);
    stereoDepth->depth.link(xoutDepth->input);


    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov4_spatial_node");

    std::string tfPrefix, resourceBaseFolder, nnPath, nnConfigPath;
    std::string camera_param_uri;
    std::string nnName(BLOB_NAME), nnConfig;  // Set your blob name for the model here
    bool syncNN, subpixel, spatial_camera;
    int confidence = 200, LRchecktresh = 5;
    std::string monoResolution = "400p";
    int monoWidth, monoHeight;
    dai::Pipeline pipeline;
    bool lrcheck, extended;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("nnConfig", "");
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("extended", false);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("nnConfig", nnConfig);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("extended", extended);
    std::cout << resourceBaseFolder << std::endl;

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {

        node->get_parameter("nnName", nnName);
        std::cout << "nn_name: " << nnName << std::endl;
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    std::cout << " Path nn: " << nnPath <<std::endl;
    
    nnConfigPath = resourceBaseFolder + "/" + nnConfig;
    std::cout << " Path config: " << nnConfigPath <<std::endl;
    
    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(lrcheck, extended, syncNN, subpixel, nnPath, nnConfigPath, confidence, LRchecktresh, monoResolution);
    
    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("color", 30, false);
    //auto detectionQueue = device.getOutputQueue("detections", 30, false);
    auto stereoQueue = device.getOutputQueue("depth", 30, false);
    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);

    auto calibrationHandler = device.readCalibration();

    dai::rosBridge::ImageConverter leftconverter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = leftconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        std::string("left/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &leftconverter, std::placeholders::_1, std::placeholders::_2),
        30,
        leftCameraInfo,
        "left");
    leftPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        std::string("right/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "right");
    rightPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter color_converter(tfPrefix + "_color_camera_optical_frame", true);
    auto colorCameraInfo = color_converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, 416, 416);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> colorPublish(
        colorQueue,
        node,
        std::string("color/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &color_converter, std::placeholders::_1, std::placeholders::_2),
        30,
        colorCameraInfo,
        "color");
    colorPublish.addPublisherCallback();

    auto depthconverter = rightconverter;//color_converter;
    auto depthCameraInfo = rightCameraInfo;//depthconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        node,
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
#if 0
    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        node,
        std::string("color/yolov4_Spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();
#endif
    rclcpp::spin(node);

    return 0;
}
