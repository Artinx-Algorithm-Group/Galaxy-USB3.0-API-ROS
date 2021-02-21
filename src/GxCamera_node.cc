#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "GxCamera.hpp"

using cv::Mat;
using cv::cvtColor;

using sensor_msgs::ImagePtr;
using cv_bridge::CvImage;

namespace {
    const double kExposureTime = 8000.0;
    const double kFrameRate = 200.0;

    // c-style string serial number for code compatibility
    // char left_cam_serial_num[] = "KE0200080465";
    // char right_cam_serial_num[] = "KE0200080462";

    bool stop_flag = false;
}

int main(int argc, char** argv){

    // Ros node init
    ros::init(argc, argv, "GxCamera_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("GxCamera/image", 1);

    GX_STATUS status;  // Camera API status

    GxCamera::Camera cam;
    status = cam.CameraInit(false);  // No software trigger
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("CameraInit fail");
        return EXIT_FAILURE;
    }

    status = cam.SetExposureTime(kExposureTime);
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Set exposure time fail");
        return EXIT_FAILURE;
    }

    status = cam.SetFrameRate(kFrameRate);
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Set frame rate fail");
        return EXIT_FAILURE;
    }

    status = cam.CameraStreamOn();
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Turn on camera stream fail");
        return EXIT_FAILURE;
    }

    Mat img;
    ImagePtr msg;
    ros::Rate loop_rate(60);
    while (nh.ok()){
        cam.GetLatestColorImg(img);
        cvtColor(img, img, cv::COLOR_RGB2BGR);

        msg = CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg);
        loop_rate.sleep();
    }
    
    cam.CameraStreamOff();
    cam.CameraClose();

    return EXIT_SUCCESS;
}