/*
 * Camera control code. This will take in camera control data, switch camera signals,
 * and show multiple cameras.
 * Camera motion/setting control operated on the primary camera, which must be configured
 * in the cameras config file.
 */

#ifndef HEADER_H
#define HEADER_H
#include "headers.h"
#endif

// Variable to store controller requests in
static uint8_t primaryCamera = 1, secondaryCamera = 2;
static int8_t pan = 0, tilt = 0;
// These dictate whether an event can happen in camera (true), or not (false).
static uchar cameraCapabilities = 0;

static double displaySize[2] = {0}; // {width, hieght}
static int primaryCameraDigit = 0, secondaryCameraDigit = 1;

// Dual camera switching
static bool updateCameras = false; // Set to true when camera numbers change.
static bool showTwoCameras, configShowTwoCameras; // Whether to show a mini camera view on display
static int miniViewerWidth = 0, miniViewerCorner = 4;
std::string dummyCameraImageLocation = "";

// Values to adjust on a running camera
static double focus = 0, focus_adj = 0;
static double exposure = 0, expo_adj = 0;
static double zoom = 1.0, zoom_adj = 0;
static int camZoomMin = 1, camZoomMax = 8;

// Configurations
std::string yamlConfigFile = "";
YAML::Node cam_conf;
bool readCameraConfigs();

// Resets the variables (zoom, exposure, etc) of a camera. Used when switching cameras.
void resetCamVars(){
  focus = 0;
  exposure = 0;
  zoom = 1;
  cameraCapabilities = 0;
  camZoomMin = 0;
  camZoomMax = 0;

  focus_adj = 0;
  expo_adj = 0;
  zoom_adj = 0;
}

std::string cvType2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');
  return r;
}

bool useMotionCommand(camera_controller::cam_ctrl::Request  &req,
      camera_controller::cam_ctrl::Response &res) {
  // res.sum = req.a + req.b;
  ROS_DEBUG("Got request; command = %d", req.cmd);

  if(primaryCamera != req.camera) {
    secondaryCamera = primaryCamera;
    primaryCamera = req.camera;
    if(!readCameraConfigs()){ // If a bad camera exists, don't try to configure cams.
      // ROS_ERROR("Bad camera! Not configuring.");
      res.error = res.BAD_CAM_NUMBER;
      return true;
    }
  }
  // Interpret data and output it to the motors and cameras.
  // If a camera does not have the hardware (hw) to accomplish a task, software
  // settings are used (sw), failng that, an image can be directly manipulated (i). These
  // options are written out in each case's header.
  res.error = 0; // Default true; if switch defaults then this is set to false.
  switch(req.cmd){
    case req.ON_OFF_TOGGLE: // Toggle power (hw & sw). No image alternative.

    break; // Camera turn on or off
    case req.IRIS_CLOSE: // Close iris (hw), alter ISO/shutter speed (sw), darken image values (i)
      expo_adj = -2.0;
    break;
    case req.IRIS_OPEN: // Open iris (hw), alter ISO/shutter speed (sw), lighten image values (i)
      expo_adj = 2.0;
    break;
    case req.FOCUS_NEAR: // Bring in focus (hw & sw). No image alternative.
      focus_adj = .01;
      ROS_INFO("Focus close");
    break;
    case req.FOCUS_FAR: // Push out focus (hw & sw). No image alternative.
      focus_adj = -.01;
      ROS_INFO("Focus far");
    break;
    case req.ZOOM_WIDE: // Zoom out (hw & sw). No image alternative.
      zoom_adj = -.03;
    break;
    case req.ZOOM_TELE: // Zoom in (hw & sw), digitally zoom image in (i).
      zoom_adj = .03;
    break;
    case req.PAN_TILT: // Pan motion for camera. Uses motors.
    break;
    case req.STOP_ALL: // Tilt motion for camera. Uses motors.
      focus_adj = 0;
      expo_adj = 0;
      zoom_adj = 0;
    break;
    default:
      res.error = res.BAD_COMMAND;
      return true;
    break;
  }
  return true;
}

bool readCameraConfigs(){
  static bool primaryCameraFound, secondaryCameraFound;
  primaryCameraFound = false; // Default false
  secondaryCameraFound = !configShowTwoCameras; // If no second camera should be shown, default this to true.
  resetCamVars(); // Reset all configuration info
  for(int i = 0; i < cam_conf.size(); i++) { // For each camera in config file
    static int camNum;
    camNum = cam_conf[i]["camera_num"].as<int>();

    if(camNum == primaryCamera) { // Primary camera settings
      primaryCameraFound = true;
      primaryCameraDigit = cam_conf[i]["device_digit"].as<int>();
      // if(primaryCameraDigit != 255) { // 255 ==> camrea is a dummy cam, aka a photo. No camera capabilities.
        cameraCapabilities = cameraCapabilities | (cam_conf[i]["shutdown"].as<bool>() << CAMERA_SHUTDOWN);
        cameraCapabilities = cameraCapabilities | (cam_conf[i]["iris"].as<bool>() << CAMERA_IRIS);
        cameraCapabilities = cameraCapabilities | (cam_conf[i]["focus"].as<bool>() << CAMERA_FOCUS);
        cameraCapabilities = cameraCapabilities | (cam_conf[i]["zoom"].as<bool>() << CAMERA_ZOOM);
        camZoomMin = cam_conf[i]["zoom_min"].as<int>();
        camZoomMax = cam_conf[i]["zoom_max"].as<int>();
        cameraCapabilities = cameraCapabilities | (cam_conf[i]["pan"].as<bool>() << CAMERA_PAN);
        cameraCapabilities = cameraCapabilities | (cam_conf[i]["tilt"].as<bool>() << CAMERA_TILT);
      // }
    } // End Primary camera settings
    else if ((configShowTwoCameras) && (camNum == secondaryCamera)){ // Secondary camera settings
      secondaryCameraFound = true;
      secondaryCameraDigit = cam_conf[i]["device_digit"].as<int>();
    } // Secondary camera settings

  } // For each camer in config file
  if(!primaryCameraFound) {
    ROS_ERROR("No configuration for camera %d!", primaryCamera);
  }
  if((configShowTwoCameras) && !secondaryCameraFound) {
    ROS_ERROR("No configuration for camera %d!", secondaryCamera);
  }

  // Change OpenCV camera device
  updateCameras = true;

  return primaryCameraFound && secondaryCameraFound;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "camera_controller");
  ros::NodeHandle n;

  // Find main config file
  std::string mainConfigFileName = "";
  if(!n.getParam("ros_configuration_file", mainConfigFileName)) {
    ROS_ERROR("No main parameters file location stored in rosparams! Maybe the file location is not stored in rosparam '/ros_configuration_file'?");
  }
  else { // Found a main config file
    // Find & load the camera config file
    YAML::Node mainConfigYAML = YAML::LoadFile(mainConfigFileName);
    static std::string camConfigFileName = mainConfigYAML["camera_config_file"].as<std::string>();
    //ROS_INFO("CONFIG STRING: %s", yamlConfigFile);
    cam_conf = YAML::LoadFile(camConfigFileName);

    // Read other config info
    configShowTwoCameras = mainConfigYAML["display_use_mini_viewer"].as<bool>();
    miniViewerWidth = mainConfigYAML["display_mini_viewer_width"].as<int>();
    miniViewerCorner = mainConfigYAML["display_mini_viewer_corner"].as<int>();
    dummyCameraImageLocation = mainConfigYAML["dummy_camera_image_file"].as<std::string>();
    displaySize[0] = mainConfigYAML["display_width"].as<int>();
    displaySize[1] = mainConfigYAML["display_height"].as<int>();

    // Now read camera configurations
    readCameraConfigs();
  }


  // Set up image publisher
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::VideoCapture mainCameraView;
  cv::VideoCapture miniCameraView;

  cv::Mat frame, firstCamInput, secondCamInput;
  sensor_msgs::ImagePtr msg;

  ros::ServiceServer service = n.advertiseService("cam_ctrl", useMotionCommand);
  // ROS_INFO("Ready.");

  // double expo = .48;
  ros::Rate loop_rate(22);
  while(n.ok()) {
    frame = cv::Mat::zeros(displaySize[1], displaySize[0], CV_8UC3);
    if (updateCameras) {
      mainCameraView.release();
      miniCameraView.release();
      // if (primaryCameraDigit != 255)
        mainCameraView.open(primaryCameraDigit);
      // if (secondaryCameraDigit != 255)
        miniCameraView.open(secondaryCameraDigit);
      // mainCameraView.enableView();
      // miniCameraView.enableView();
      updateCameras = false;
      if (!mainCameraView.isOpened()) {
        ROS_ERROR("Could not swtich to primary camera: %d.", primaryCameraDigit);
      }
      else if ((configShowTwoCameras) && !miniCameraView.isOpened()){
        ROS_ERROR("Could not swtich to secondary camera: %d.", secondaryCameraDigit);
        showTwoCameras = false;
      }
      else showTwoCameras = configShowTwoCameras; // Both cams work; show second one?
      // Setup for the cameras
      if(!mainCameraView.set(CV_CAP_PROP_FPS, 25)) ROS_ERROR("Could not adjust fps.");
      if (focus != 0)
        if(!mainCameraView.set(CV_CAP_PROP_AUTOFOCUS, 0)) ROS_ERROR("Could not lock focus.");
      if(!mainCameraView.set(CV_CAP_PROP_AUTO_EXPOSURE, 1)) ROS_ERROR("Could not set exposure.");
    }

    // ========== Grab and Resize Frames ==========
    if (primaryCameraDigit != 255)
      mainCameraView >> firstCamInput;
    else
      firstCamInput = cv::imread(dummyCameraImageLocation, CV_LOAD_IMAGE_COLOR);
    if (secondaryCameraDigit != 255)
      miniCameraView >> secondCamInput;
    else
      secondCamInput = cv::imread(dummyCameraImageLocation, CV_LOAD_IMAGE_COLOR);
    // ROS_INFO("Display: w: %f; h: %f", displaySize[0], displaySize[1]);
    // ROS_INFO("Image: w: %f; h: %f", firstCamInput.size().width, firstCamInput.size().height);
    // Same ratio of width:height ==> equal resize
    if ((((float)displaySize[0]) / ((float)displaySize[1])) == (((float)firstCamInput.cols) / ((float)firstCamInput.rows))) {
      resize(firstCamInput, firstCamInput, cv::Size(displaySize[0], displaySize[1]));
      // ROS_INFO("Resize: standard");
    }
    // Lower ratio of width:height than display ==> resize based on height
    else if ((((float)displaySize[0]) / ((float)displaySize[1])) > (((float)firstCamInput.cols) / ((float)firstCamInput.rows))) {
      resize(firstCamInput, firstCamInput, cv::Size(), (((float)displaySize[1])/((float)firstCamInput.rows)), (((float)displaySize[1])/((float)firstCamInput.rows)));
      // ROS_INFO("Resize: height ratio: %f", (((float)displaySize[1])/((float)firstCamInput.rows)));
    }
    // Higher ratio of width:height than display ==> resize based on width
    else {
      resize(firstCamInput, firstCamInput, cv::Size(), (((float)displaySize[0])/((float)firstCamInput.cols)), (((float)displaySize[0])/((float)firstCamInput.cols)));
      // ROS_INFO("Resize: width ratio: %f", (((float)displaySize[0])/((float)firstCamInput.cols)));
    }

    // ========== Adjust Focus ==========
    focus += focus_adj;
    if (focus <= 0) focus = 0;
    else if (focus > 1) focus = 1;
    // If camera is capable of focusing:
    if((((int)cameraCapabilities & (1 << CAMERA_FOCUS)) != 0) && (focus_adj != 0)) {
      mainCameraView.set(CV_CAP_PROP_FOCUS, focus);
      // focusTextToDisplay += std::to_string(focus);
    }
    // Camera cannot focus! software option?
    else if (focus_adj != 0) { // This is where I would put my image enhancing code,
      // If I had some.
      focus = 0; // Show on display that this is not changing
    }

    // ========== Adjust Zoom ==========
    /* No hardware zoom works in this version of OpenCV. (Add hardware zoom
     * code to the block of code that runs if zoom is < 1.3! NOT the
     * if(zoom > 1.3) {...} block, use the else {...} block after it.) */
    zoom += zoom_adj;
    // If hardware zoom is available, skip software zoom by setting zoom = 1.29.
    // ROS_INFO("");
    if (zoom < ZOOM_STOP_MIN_VAL || ((int)cameraCapabilities & (1 << CAMERA_ZOOM) != 0))
      zoom = ZOOM_STOP_MIN_VAL; // Zoom level under 1.3 is ignored; this is = to no zoom at all
    else if (zoom > 8) zoom = 8;
    // Zoom level under around 1.3 causes a crash since it thinks that a number div by 1 = 0.
    if(zoom > 1.3) {
      double zoomedImageCols, zoomedImageRows, zoomedImagePosX, zoomedImagePosY, zoomX, zoomY;
      zoomX = zoom * ((double)firstCamInput.cols / (double)frame.cols);
      zoomY = zoom * ((double)firstCamInput.rows / (double)frame.rows);
      // ROS_INFO("zoom values: x: %f, y: %f", zoomX, zoomY);
// Checks if black bars are needed for rows after zooming in:
      if(firstCamInput.cols * zoom < frame.cols) { // Keep & scale column black bars while zooming
        // ROS_INFO("Keeping COL black bars.");
        zoomedImagePosX = (frame.cols - (firstCamInput.cols * zoom)) / 2;
        zoomedImageCols = firstCamInput.cols * zoom;

        if(firstCamInput.rows * zoom < frame.rows) { // should not need to keep both bars
        }
        else { // Tall image; zoom y
          // ROS_INFO("Removing ROW black bars.");
          zoomedImagePosY = 0;
          zoomedImageRows = frame.rows;
          // ROS_INFO("Grabbing ROI");
          // Rectangle should not zoom in on x, but should on y
          cv::Mat zoomedImage(firstCamInput, cv::Rect(0,
              ((firstCamInput.rows / 2) - (firstCamInput.rows / (2 * zoomY))),
              (firstCamInput.cols), (firstCamInput.rows / (zoomY))));
          // Resize the ROI; x needs to be increased by zoom level, y needs to be brought to screen size.
          // ROS_INFO("resizing");
          resize(zoomedImage, zoomedImage, cv::Size(zoomedImageCols, zoomedImageRows));
          // Place the image:
          // x pos = (frame size - zoomed image size) / 2 = black bar size, y pos = 0
          // x width = image width * zoom, y height = frame height
          // ROS_INFO("Placing inset image, x, y: %f, %f...", zoomedImagePosX, zoomedImagePosY);
          cv::Mat insetImage(frame, cv::Rect(zoomedImagePosX, zoomedImagePosY, zoomedImageCols, zoomedImageRows));
          // ROS_INFO("Done; copying image.");
          zoomedImage.copyTo(insetImage);
        }
      }
      else { // Removes column black bars while zooming
        // ROS_INFO("Removing COL black bars.");
        zoomedImagePosX = 0;
        zoomedImageCols = frame.cols;

        if(firstCamInput.rows * zoom < frame.rows) { // Image longer than screen; scale row black bars while zooming
          // ROS_INFO("Keeping ROW black bars.");
          zoomedImagePosY = (frame.rows - (firstCamInput.rows * zoom)) / 2;
          zoomedImageRows = firstCamInput.rows * zoom;
          // ROS_INFO("Grabbing ROI");
          // Rectangle should zoom in on x, but should not on y
          cv::Mat zoomedImage(firstCamInput, cv::Rect(
              ((firstCamInput.cols / 2) - (firstCamInput.cols / (2 * zoomX))),
              0, (firstCamInput.cols / zoomX), (firstCamInput.rows)));
          // Resize the ROI; x needs to be brought to screen size, y needs to be increased by zoom level
          // ROS_INFO("resizing");
          resize(zoomedImage, zoomedImage, cv::Size(zoomedImageCols, zoomedImageRows));
          // Place the image:
          // x pos = 0, y pos = (frame size - zoomed image size) / 2 = black bar size
          // x width = frame width, y height = image height * zoom
          // ROS_INFO("Placing inset image, x, y: %f, %f...", zoomedImagePosX, zoomedImagePosY);
          cv::Mat insetImage(frame, cv::Rect(zoomedImagePosX, zoomedImagePosY, zoomedImageCols, zoomedImageRows));
          // ROS_INFO("Done; copying image.");
          zoomedImage.copyTo(insetImage);
        }
        else {// Zooms in both x and y
          // ROS_INFO("Removing ROW black bars.");
          zoomedImagePosY = 0;
          zoomedImageRows = frame.rows;
          // ROS_INFO("Grabbing ROI");
          // Rectangle should zoom in on x and y
          cv::Mat zoomedImage(firstCamInput, cv::Rect(
              ((firstCamInput.cols / 2) - (firstCamInput.cols / (2 * zoomX))),
              ((firstCamInput.rows / 2) - (firstCamInput.rows / (2 * zoomY))),
              (firstCamInput.cols / zoomX), (firstCamInput.rows / zoomY)));
          // Resize the ROI; x & y need to be brought to frame size
          // ROS_INFO("resizing");
          resize(zoomedImage, zoomedImage, cv::Size(zoomedImageCols, zoomedImageRows));
          // Place the image:
          // x pos = 0, y pos = 0
          // x width = frame width, y height = frame height
          // ROS_INFO("Placing inset image, x, y: %f, %f...", zoomedImagePosX, zoomedImagePosY);
          cv::Mat insetImage(frame, cv::Rect(zoomedImagePosX, zoomedImagePosY, zoomedImageCols, zoomedImageRows));
          // ROS_INFO("Done; copying image.");
          zoomedImage.copyTo(insetImage);
        }
      }
      // zoomTextToDisplay += std::to_string(zoom);
    }
    else { // Don't zoom; zoom levels around 1 produce errors.
    // IF IMPLEMENTING HARDWARE ZOOM, ADD IT HERE, AND EDIT THE LINE WHICH
    // STORES ZOOM LEVEL TO BE DISPLAYED!
      cv::Mat insetImage(frame, cv::Rect(
          (frame.cols - firstCamInput.cols) / 2,
          (frame.rows - firstCamInput.rows) / 2,
          firstCamInput.cols, firstCamInput.rows));
      firstCamInput.copyTo(insetImage);
    }

    // ========= Adjust Image Exposure ==========
    // Camera exposure settings
    if (((int)cameraCapabilities & (1 << CAMERA_ZOOM) != 0)){
      // Camera exposure settings (similar to camera focus)
      // Also see line containing "mainCameraView.set(CV_CAP_PROP_AUTO_EXPOSURE, 1)"
    }
    else { // Software exposure
      exposure += expo_adj;
      if (exposure > 255) exposure = 255;
      else if (exposure < -255) exposure = -255;
      frame.convertTo(frame, CV_8UC3, 1, exposure);
      // expoTextToDisplay += std::to_string(exposure);
      // memset(buffer, 0, sizeof(buffer));
      // snprintf(buffer, sizeof(buffer), "%g", exposure);
      // expoTextToDisplay += buffer;

// std::string strObj4(buffer);
    }

    // ========== Write Everything to Display ==========
    // Second camera is resized then written to frame
    int miniViewerHeight = (double)((double)miniViewerWidth / (double)secondCamInput.cols) * (double)secondCamInput.rows;
    cv::resize(secondCamInput, secondCamInput, cv::Size(miniViewerWidth, miniViewerHeight), CV_INTER_AREA);
    int miniViewerX, miniViewerY;
    switch(miniViewerCorner) {
      case 1: // Top left corner
        miniViewerX = MINI_VIEWER_PADDING;
        miniViewerY = MINI_VIEWER_PADDING;
      break;
      case 2: // Top right corner
        miniViewerX = frame.cols - (miniViewerWidth + MINI_VIEWER_PADDING);
        miniViewerY = MINI_VIEWER_PADDING;
      break;
      case 3: // Bottom left corner
        miniViewerX = MINI_VIEWER_PADDING;
        miniViewerY = frame.rows - (miniViewerHeight + MINI_VIEWER_PADDING);
      break;
      case 4: // Bottom right corner
        miniViewerX = frame.cols - (miniViewerWidth + MINI_VIEWER_PADDING);
        miniViewerY = frame.rows - (miniViewerHeight + MINI_VIEWER_PADDING);
      break;
    }
    cv::Mat insetImage(frame, cv::Rect(miniViewerX, miniViewerY, miniViewerWidth, miniViewerHeight));
    secondCamInput.copyTo(insetImage);

    // Write other info on frame
    static char titleBuffer[32]; // For putting text on screen
    memset(titleBuffer, 0, sizeof(titleBuffer));
    if (zoom_adj != 0) {
      std::string zoomTextToDisplay = "Zoom: ";
      if (zoom == ZOOM_STOP_MIN_VAL)
        zoomTextToDisplay += "1.00";
      else {
        snprintf(titleBuffer, sizeof(titleBuffer), "%.2f", zoom);
        zoomTextToDisplay += titleBuffer;
      }
      // zoomTextToDisplay += titleBuffer;
      cv::Size textSize = cv::getTextSize(zoomTextToDisplay, CV_FONT_HERSHEY_PLAIN, 1, 2, 0);
      putText(frame, zoomTextToDisplay, cvPoint((frame.cols / 2) - (textSize.width), (frame.rows / 2) - (textSize.height / 2)),
          CV_FONT_HERSHEY_PLAIN, 1, cvScalar(0,0,250), 2, CV_AA);
      }
    else if (focus_adj != 0) {
      std::string focusTextToDisplay = "Focus: ";
      snprintf(titleBuffer, sizeof(titleBuffer), "%.2f", focus);
      focusTextToDisplay += titleBuffer;
      cv::Size textSize = cv::getTextSize(focusTextToDisplay, CV_FONT_HERSHEY_PLAIN, 1, 2, 0);
      putText(frame, focusTextToDisplay, cvPoint((frame.cols / 2) - (textSize.width), (frame.rows / 2) - (textSize.height / 2)),
          CV_FONT_HERSHEY_PLAIN, 1, cvScalar(0,0,250), 2, CV_AA);
      }
    else if (expo_adj != 0) {
      std::string expoTextToDisplay = "Exposure: ";
      snprintf(titleBuffer, sizeof(titleBuffer), "%.2f", exposure);
      expoTextToDisplay += titleBuffer;
      cv::Size textSize = cv::getTextSize(expoTextToDisplay, CV_FONT_HERSHEY_PLAIN, 1, 2, 0);
      putText(frame, expoTextToDisplay, cvPoint((frame.cols / 2) - (textSize.width), (frame.rows / 2) - (textSize.height / 2)),
          CV_FONT_HERSHEY_PLAIN, 1, cvScalar(0,0,250), 2, CV_AA);
      }

    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
    ROS_INFO("Sending frame; fw: %d, fh: %d. Type: %s", frame.cols, frame.rows, cvType2str(frame.type()).c_str());
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    // expo += .0001;
    // if(expo >= .51) expo = 0;

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
