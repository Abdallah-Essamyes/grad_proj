// ============================================================
// yolov4_rtabmap_publisher.cpp
//
// ADDED vs yolov4_spatial_publisher.cpp:
//   1. Full-resolution color video stream → "color/image" (1080p BGR) for RTAB-Map
//      (the 416×416 YOLO passthrough is renamed to "color/preview/image")
//   2. Rectified right mono image → "right/image_rect" + "right/camera_info"
//      so RTAB-Map rgbd_sync can use mono as an alternative channel
//   3. IMU node (ACCELEROMETER_RAW 200Hz + GYROSCOPE_RAW 200Hz) → "imu"
//      with optional inline axis transformation controlled by
//      `transform_imu_to_base_frame` parameter (mirrors test_stereo_inertial_node)
//      Transformation: BNO086 sensor frame → ROS REP-103 (X=forward, Y=left, Z=up)
//        ros_x = -sensor_z,  ros_y = -sensor_x,  ros_z = sensor_y
//   4. Depth remains aligned to CAM_A (color camera) so RTAB-Map rgbd_sync
//      only needs color/image + stereo/depth (same intrinsics, no frame mismatch)
//
// Topics published:
//   color/image              — full 1080p color (NEW, for RTAB-Map)
//   color/preview/image      — 416×416 YOLO passthrough (was color/image)
//   color/yolov4_Spatial_detections — SpatialDetectionArray (unchanged)
//   stereo/depth             — depth aligned to color camera  (unchanged)
//   right/image_rect         — rectified right mono (NEW)
//   right/camera_info        — right camera intrinsics (NEW, also via stereo/depth)
//   imu                      — IMU with optional axis correction (NEW)
// ============================================================

#include <atomic>
#include <cstdio>
#include <deque>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

const std::vector<std::string> label_map = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

// ---------------------------------------------------------------------------
// Pipeline factory
// ---------------------------------------------------------------------------
dai::Pipeline createPipeline(bool syncNN, bool subpixel, const std::string& nnPath,
                              int confidence, int LRchecktresh, const std::string& resolution,
                              bool depthAlignToMono) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;

    // ── Camera nodes ──────────────────────────────────────────────────────
    auto colorCam  = pipeline.create<dai::node::ColorCamera>();
    auto monoLeft  = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo    = pipeline.create<dai::node::StereoDepth>();

    // ── Neural network ────────────────────────────────────────────────────
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();

    // ── IMU (ADDED) ───────────────────────────────────────────────────────
    auto imu     = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    // Publish each packet immediately to minimise IMU latency
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(5);
    imu->out.link(xoutImu->input);

    // ── XLink outputs ─────────────────────────────────────────────────────
    auto xoutPreview   = pipeline.create<dai::node::XLinkOut>();  // 416×416 YOLO passthrough
    auto xoutColor     = pipeline.create<dai::node::XLinkOut>();  // ISP-scaled color → RTAB-Map
    auto xoutRight     = pipeline.create<dai::node::XLinkOut>();  // rectified right mono
    auto xoutDepth     = pipeline.create<dai::node::XLinkOut>();  // depth aligned to CAM_A (YOLO+color SLAM)
    auto xoutDepthRight = pipeline.create<dai::node::XLinkOut>(); // depth aligned to CAM_C (greyscale SLAM)
    auto xoutNN        = pipeline.create<dai::node::XLinkOut>();

    xoutPreview->setStreamName("preview");
    xoutColor->setStreamName("color_video");
    xoutRight->setStreamName("right_rect");
    xoutDepth->setStreamName("depth");           // CAM_A-aligned, full rate
    xoutDepthRight->setStreamName("depth_right"); // CAM_C-aligned, full rate
    xoutNN->setStreamName("detections");

    // ── Color camera ──────────────────────────────────────────────────────
    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    // setIspScale(2, 3): downscale 1920×1080 → 1280×720, preserving full sensor FOV.
    // Unlike setVideoSize (crops then scales), ISP scale is a proper 2/3 downscale.
    // Preview (416×416) must fit within ISP output (1280×720) — 416 < 720 ✓
    colorCam->setIspScale(2, 3);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // ── Mono resolution ───────────────────────────────────────────────────
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    // ── Stereo depth (primary) — aligned to CAM_A for YOLO spatial coords + color SLAM ────
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(subpixel);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);  // 1280×720

    // ── Stereo depth (right) — aligned to CAM_C for greyscale SLAM ────────────────
    // Second independent StereoDepth node with the same settings but aligned to
    // CAM_C (right mono), producing depth at the same resolution as right/image_rect.
    // This lets rgbd_sync use right/image_rect + stereo/depth_right with perfectly
    // matching dimensions (e.g. 640×480 at 480p) and correct CAM_C intrinsics.
    auto stereoRight = pipeline.create<dai::node::StereoDepth>();
    stereoRight->initialConfig.setConfidenceThreshold(confidence);
    stereoRight->setRectifyEdgeFillColor(0);
    stereoRight->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereoRight->setSubpixel(subpixel);
    stereoRight->setDepthAlign(dai::CameraBoardSocket::CAM_C);  // mono res

    // ── Detection network ─────────────────────────────────────────────────
    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // ── Wiring ────────────────────────────────────────────────────────────
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Second stereo node for greyscale SLAM (CAM_C-aligned depth)
    monoLeft->out.link(stereoRight->left);
    monoRight->out.link(stereoRight->right);
    stereoRight->depth.link(xoutDepthRight->input);

    // YOLO preview passthrough
    colorCam->preview.link(spatialDetectionNetwork->input);
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xoutPreview->input);
    else
        colorCam->preview.link(xoutPreview->input);
    spatialDetectionNetwork->out.link(xoutNN->input);

    // Depth → YOLO spatial (internal, for 3D coord computation)
    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    // Depth direct output — wired from stereo->depth, NOT from passthroughDepth.
    // passthroughDepth only emits when YOLO inference completes (throttled by NN).
    // Direct wiring publishes at full stereo camera rate for RTAB-Map & depth_to_cloud.
    stereo->depth.link(xoutDepth->input);

    // Full-FOV ISP-scaled color (1280×720) → RTAB-Map
    colorCam->video.link(xoutColor->input);

    // ADDED: rectified right mono → RTAB-Map (also useful for mono odometry fallback)
    stereo->rectifiedRight.link(xoutRight->input);

    return pipeline;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov4_rtabmap_node");

    // ── Parameters ────────────────────────────────────────────────────────
    std::string tfPrefix, resourceBaseFolder, nnPath, camera_param_uri;
    std::string nnName(BLOB_NAME);
    bool syncNN, subpixel;
    int confidence = 200, LRchecktresh = 5;
    std::string monoResolution = "400p";

    // ADDED IMU parameters
    bool transformImuToBaseFrame = false;
    int imuModeParam = 1;
    double linearAccelCovariance = 0.0;
    double angularVelCovariance  = 0.02;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseFolder", "");
    // ADDED
    node->declare_parameter("transform_imu_to_base_frame", false);
    node->declare_parameter("imuMode", 1);
    node->declare_parameter("linearAccelCovariance", 0.0);
    node->declare_parameter("angularVelCovariance", 0.02);
    node->declare_parameter("depth_align_to_mono", false);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    // ADDED
    node->get_parameter("transform_imu_to_base_frame", transformImuToBaseFrame);
    node->get_parameter("imuMode", imuModeParam);
    node->get_parameter("linearAccelCovariance", linearAccelCovariance);
    node->get_parameter("angularVelCovariance", angularVelCovariance);

    bool depthAlignToMono = false;
    node->get_parameter("depth_align_to_mono", depthAlignToMono);
    RCLCPP_INFO(node->get_logger(), "depth_align_to_mono: %s",
                depthAlignToMono ? "TRUE" : "FALSE");

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resource folder containing NNBlob in 'resourceBaseFolder'");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(!nnParam.empty() && nnParam != "x") {
        nnName = nnParam;
    }

    RCLCPP_INFO(node->get_logger(), "transform_imu_to_base_frame: %s",
                transformImuToBaseFrame ? "TRUE" : "FALSE");

    nnPath = resourceBaseFolder + "/" + nnName;
    dai::Pipeline pipeline = createPipeline(syncNN, subpixel, nnPath, confidence, LRchecktresh, monoResolution, depthAlignToMono);
    dai::Device device(pipeline);

    // ── Mono resolution → pixel dimensions ───────────────────────────────
    int monoWidth, monoHeight;
    if(monoResolution == "720p")      { monoWidth = 1280; monoHeight = 720; }
    else if(monoResolution == "400p") { monoWidth = 640;  monoHeight = 400; }
    else if(monoResolution == "800p") { monoWidth = 1280; monoHeight = 800; }
    else if(monoResolution == "480p") { monoWidth = 640;  monoHeight = 480; }
    else {
        RCLCPP_ERROR(node->get_logger(), "Invalid monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640; monoHeight = 480;
    }

    // ── Output queues ─────────────────────────────────────────────────────
    auto previewQueue     = device.getOutputQueue("preview",      30,  false);
    auto colorVideoQueue  = device.getOutputQueue("color_video",  30,  false);
    auto rightRectQueue   = device.getOutputQueue("right_rect",   30,  false);
    auto imuQueue         = device.getOutputQueue("imu",          200, false);
    auto detectionQueue   = device.getOutputQueue("detections",   30,  false);
    auto depthQueue       = device.getOutputQueue("depth",        30,  false);       // CAM_A
    auto depthRightQueue  = device.getOutputQueue("depth_right",  30,  false);      // CAM_C

    // ── Converters ────────────────────────────────────────────────────────
    // Color camera (CAM_A) — used for both full-res color and depth camera_info.
    // Color camera (CAM_A): ISP-scaled to 1280×720 via setIspScale(2,3) — full FOV, no crop.
    // Depth aligned to CAM_A (color mode) will also output at 1280×720.
    // In greyscale_slam mode depth is aligned to CAM_C, but colorCameraInfo is still
    // used for the color/image publisher (camera_info follows ISP output dimensions).
    dai::rosBridge::ImageConverter colorConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto colorCameraInfo = colorConverter.calibrationToCameraInfo(
        calibrationHandler, dai::CameraBoardSocket::CAM_A, 1280, 720);

    // Right mono camera (CAM_C)
    dai::rosBridge::ImageConverter rightConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightConverter.calibrationToCameraInfo(
        calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

    // ── Publisher: color/image (640×360, full FOV via setIspScale) → RTAB-Map ─────
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> colorVideoPublish(
        colorVideoQueue, node,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &colorConverter, std::placeholders::_1, std::placeholders::_2),
        30, colorCameraInfo, "color");
    colorVideoPublish.addPublisherCallback();

    // ── Publisher: YOLO 416×416 preview passthrough ───────────────────────
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> previewPublish(
        previewQueue, node,
        std::string("color/preview/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &colorConverter, std::placeholders::_1, std::placeholders::_2),
        30, colorCameraInfo, "color/preview");
    previewPublish.addPublisherCallback();

    // ── Publisher: depth aligned to CAM_A (1280×720) — for YOLO spatial + color SLAM ────
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        depthQueue, node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &colorConverter, std::placeholders::_1, std::placeholders::_2),
        30, colorCameraInfo, "stereo");
    depthPublish.addPublisherCallback();

    // ── Publisher: depth aligned to CAM_C (mono res) — for greyscale RTAB-Map ──────────
    // Dimensions match right/image_rect exactly → no RTAB-Map size assertion crash.
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthRightPublish(
        depthRightQueue, node,
        std::string("stereo/depth_right"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &rightConverter, std::placeholders::_1, std::placeholders::_2),
        30, rightCameraInfo, "stereo_right");
    depthRightPublish.addPublisherCallback();

    // ── Publisher: YOLO spatial detections ────────────────────────────────
    dai::rosBridge::SpatialDetectionConverter detConverter(
        tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue, node,
        std::string("color/yolov4_Spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg,
                  &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();

    // ── Publisher: rectified right mono (ADDED) ───────────────────────────
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightRectQueue, node,
        std::string("right/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &rightConverter, std::placeholders::_1, std::placeholders::_2),
        30, rightCameraInfo, "right");
    rightPublish.addPublisherCallback();

    // ── Publisher: IMU with optional axis transformation (ADDED) ──────────
    // Frame ID: if transformImuToBaseFrame=true, data is in oak-d-base-frame (ROS REP-103).
    //           if false, data is in oak_imu_frame (raw BNO086 sensor frame).
    std::string imuFrameId = transformImuToBaseFrame
        ? (tfPrefix + "-d-base-frame")
        : (tfPrefix + "_imu_frame");
    RCLCPP_INFO(node->get_logger(), "IMU frame_id: %s", imuFrameId.c_str());

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    dai::rosBridge::ImuConverter imuConverter(imuFrameId, imuMode,
                                              linearAccelCovariance, angularVelCovariance);

    // Inline transformation counter for debug logging
    std::atomic<int> transformCount{0};

    // Conversion lambda — mirrors the implementation in stereo_inertial_publisher.cpp.
    // When transform_imu_to_base_frame=true, applies rotation:
    //   R = [[ 0, 0,-1],[-1, 0, 0],[ 0, 1, 0]]
    //   ros_x = -sensor_z,  ros_y = -sensor_x,  ros_z = sensor_y
    // This converts the BNO086 sensor frame to ROS REP-103 (X=forward, Y=left, Z=up).
    auto imuConversionFunc = [&imuConverter, transformImuToBaseFrame, &transformCount, node](
        std::shared_ptr<dai::IMUData> inData,
        std::deque<sensor_msgs::msg::Imu>& outImuMsgs) {
        imuConverter.toRosMsg(inData, outImuMsgs);

        if(transformImuToBaseFrame) {
            int count = ++transformCount;
            if(count <= 5) {
                RCLCPP_INFO(node->get_logger(),
                            "[IMU transform #%d] Applying BNO086→REP-103 axis correction", count);
            }

            for(auto& imuMsg : outImuMsgs) {
                // Linear acceleration
                double sx = imuMsg.linear_acceleration.x;
                double sy = imuMsg.linear_acceleration.y;
                double sz = imuMsg.linear_acceleration.z;
                imuMsg.linear_acceleration.x = sz;
                imuMsg.linear_acceleration.y = -sx;
                imuMsg.linear_acceleration.z = -sy;

                // Angular velocity (same rotation matrix, right-hand rule preserved)
                double gx = imuMsg.angular_velocity.x;
                double gy = imuMsg.angular_velocity.y;
                double gz = imuMsg.angular_velocity.z;
                imuMsg.angular_velocity.x = gz;
                imuMsg.angular_velocity.y = -gx;
                imuMsg.angular_velocity.z = -gy;

                // Transform covariance: C_ros = R × C_sensor × R^T
                // R = [[0,0,1],[-1,0,0],[0,-1,0]]
                auto transformCov = [](std::array<double, 9>& cov) {
                    std::array<double, 9> o = cov;
                    cov[0] =  o[8]; cov[1] = -o[6]; cov[2] = -o[7];
                    cov[3] = -o[2]; cov[4] =  o[0]; cov[5] =  o[1];
                    cov[6] = -o[5]; cov[7] =  o[3]; cov[8] =  o[4];
                };
                transformCov(imuMsg.linear_acceleration_covariance);
                transformCov(imuMsg.angular_velocity_covariance);
            }
        }
    };

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue, node,
        std::string("imu"),
        imuConversionFunc,
        200,  // queue depth matches imuQueue size
        "",
        "imu");
    imuPublish.addPublisherCallback();

    RCLCPP_INFO(node->get_logger(),
                "yolov4_rtabmap_node ready — publishing color/image (1280×720), "
                "stereo/depth (CAM_A-aligned), stereo/depth_right (CAM_C-aligned), "
                "right/image_rect, imu, color/yolov4_Spatial_detections");

    rclcpp::spin(node);
    return 0;
}
