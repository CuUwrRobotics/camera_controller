#include <ros/ros.h>
#include "camera_controller/cam_ctrl.h"
#include <yaml-cpp/yaml.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Camera's configured capabilities (array indexes):
#define CAMERA_SHUTDOWN 0
#define CAMERA_IRIS 1
#define CAMERA_FOCUS 2
#define CAMERA_ZOOM 3
#define CAMERA_PAN 4
#define CAMERA_TILT 5

#define delay(a) usleep(a*1000)

#define ZOOM_STOP_MIN_VAL 1.29
#define MINI_VIEWER_PADDING 10
