#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/highgui/highgui.hpp>

#include "GxCamera.hpp"

using std::endl;
using std::string;
using std::stringstream;
using std::vector;

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

string IntrinsicsToString(vector<double> v);
string DistortionToString(vector<double> v);
boost::array<double, 9> IntrinsicsToBoostArray(vector<double> v);

int main(int argc, char** argv){

    // Ros node init
    ros::init(argc, argv, "GxCamera_node");
    ros::NodeHandle nh;

    ros::Publisher cam_info_pub = nh.advertise<CameraInfo>("GxCamera/camera_info", 10);;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("GxCamera/image", 1);

    // Load calibration parameters
    int height = 0;
    int width = 0;
    nh.getParam("gxcam_height", height);
    nh.getParam("gxcam_width", width);

    vector<double> intrinsics;
    nh.getParam("gxcam_intrinsics", intrinsics);
    ROS_INFO_STREAM("Load camera intrinsics: " << endl << IntrinsicsToString(intrinsics));
    boost::array<double, 9> intrinsics_array = IntrinsicsToBoostArray(intrinsics);

    vector<double> distortion;
    nh.getParam("gxcam_distortion", distortion);
    ROS_INFO_STREAM("Load camera distortion params: " << endl << DistortionToString(distortion));

    string distortion_model;
    nh.getParam("gxcam_distortion_model", distortion_model);
    ROS_INFO_STREAM("Load camera distortion model: " << distortion_model);

    // Load camera setting parameters
    double exp_time = 0.0;
    double frame_rate = 0.0;
    nh.getParam("gxcam_expsure_time", exp_time);
    nh.getParam("gxcam_frame_rate", frame_rate);

    GX_STATUS status;  // Camera API status

    GxCamera::Camera cam;
    status = cam.CameraInit(false);  // No software trigger
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("CameraInit fail");
        return EXIT_FAILURE;
    }
    
    ROS_INFO("Load target exposure time: %f", exp_time);
    status = cam.SetExposureTime(exp_time);
    if(status != GX_STATUS_SUCCESS){
        ROS_ERROR("Set exposure time fail");
        return EXIT_FAILURE;
    }
    
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

    ros::Rate loop_rate(60);
    while (nh.ok()){
        Mat img;
        cam.GetLatestColorImg(img);
        cvtColor(img, img, cv::COLOR_RGB2BGR);

        ros::Time current_time = ros::Time::now();

        ImagePtr msg;
        msg = CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = current_time;

        CameraInfo cam_info;
        cam_info.header.frame_id = "GxCamera_frame";
        cam_info.header.stamp = current_time;
        cam_info.width = width;
        cam_info.height = height;
        cam_info.distortion_model = distortion_model;
        cam_info.D = distortion;
        cam_info.K = intrinsics_array;

        img_pub.publish(msg);
        cam_info_pub.publish(cam_info);
        
        loop_rate.sleep();
    }
    
    cam.CameraStreamOff();
    cam.CameraClose();

    return EXIT_SUCCESS;
}

boost::array<double, 9> IntrinsicsToBoostArray(vector<double> v){
    boost::array<double, 9> a;
    a[0] = v[0];
    a[1] = v[1];
    a[2] = v[2];
    a[3] = v[3];
    a[4] = v[4];
    a[5] = v[5];
    a[6] = v[6];
    a[7] = v[7];
    a[8] = v[8];

    return a;
}

string IntrinsicsToString(vector<double> v){
    stringstream ss;
    ss << v[0] << " " << v[1] << " " << v[2] << endl
       << v[3] << " " << v[4] << " " << v[5] << endl
       << v[6] << " " << v[7] << " " << v[8] << endl;

    return ss.str();
}

string DistortionToString(vector<double> v){
    stringstream ss;
    ss << v[0] << v[1] << v[2] << v[3] << v[4];

    return ss.str();
}