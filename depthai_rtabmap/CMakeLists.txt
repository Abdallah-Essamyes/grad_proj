cmake_minimum_required(VERSION 3.10.2)
project(depthai_rtabmap VERSION 1.0.0 LANGUAGES CXX C)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(tiny_yolo_v4_blob_name "yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob")

add_executable(yolov4_rtabmap_node src/yolov4_rtabmap_publisher.cpp)

target_link_libraries(yolov4_rtabmap_node
    depthai::core
    opencv_imgproc
    opencv_highgui
    /lib/x86_64-linux-gnu/libcurl.so.4
)

ament_target_dependencies(yolov4_rtabmap_node
    rclcpp
    depthai_ros_msgs
    depthai_bridge
    camera_info_manager
    cv_bridge
    sensor_msgs
    stereo_msgs
    std_msgs
)

target_compile_definitions(yolov4_rtabmap_node PRIVATE
    BLOB_NAME="${tiny_yolo_v4_blob_name}"
)

install(TARGETS yolov4_rtabmap_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py"
)

ament_package()
