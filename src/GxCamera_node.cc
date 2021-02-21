#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/highgui/highgui.hpp>

#include "GxCamera.hpp"

using cv::Mat;
using cv::cvtColor;

using sensor_msgs::ImagePtr;
using sensor_msgs::CameraInfo;
using cv_bridge::CvImage;

namespace {
    // const double kExposureTime = 8000.0;
    // const double kFrameRate = 200.0;

    // c-style string serial number for code compatibility
    // char left_cam_serial_num[] = "KE0200080465";
    // char right_cam_serial_num[] = "KE0200080462";
}

int main(int argc, char** argv){

    // Ros node init
    ros::init(argc, argv, "GxCamera_node");
    ros::NodeHandle nh;

    ros::Publisher cam_info_pub = nh.advertise<CameraInfo>("GxCamera/camera_info", 10);;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("GxCamera/image", 1);

    GX_STATUS status;  // Camera API status

    GxCamera::Camera cam;
    status = cam.CameraInit(false);  // No software trigger
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("CameraInit fail");
        return EXIT_FAILURE;
    }

    double exp_time = 0.0;
    nh.getParam("gxcam_expsure_time", exp_time);
    ROS_INFO("Load target exposure time: %f", exp_time);
    status = cam.SetExposureTime(exp_time);
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Set exposure time fail");
        return EXIT_FAILURE;
    }

    double frame_rate = 0.0;
    nh.getParam("gxcam_frame_rate", frame_rate);
    ROS_INFO("Load target frame rate: %f", frame_rate);
    status = cam.SetFrameRate(frame_rate);
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Set frame rate fail");
        return EXIT_FAILURE;
    }

    status = cam.CameraStreamOn();
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Turn on camera stream fail");
        return EXIT_FAILURE;
    }

    CameraInfo cam_info;

    Mat img;
    ros::Rate loop_rate(60);
    while (nh.ok()){
        cam.GetLatestColorImg(img);
        cvtColor(img, img, cv::COLOR_RGB2BGR);

        ros::Time current_time = ros::Time::now();

        ImagePtr msg;
        msg = CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = current_time;

        img_pub.publish(msg);
        
        loop_rate.sleep();
    }
    
    cam.CameraStreamOff();
    cam.CameraClose();

    return EXIT_SUCCESS;
}