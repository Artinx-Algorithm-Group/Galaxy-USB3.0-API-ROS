/**
 * @file    GxCamera.hpp
 * @brief   Class declaration for DaHeng usb3.0 camera
 * @author  Jing Yonglin
 * @mail:   11712605@mail.sustech.edu.cn
 *          yonglinjing7@gmail.com
*/

#ifndef _GXCAMERA_HPP_
#define _GXCAMERA_HPP_

#include <opencv2/opencv.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"

namespace GxCamera {

enum class ImageFormatConvertStatus;
class GxException;

class Camera{

public:

    GX_STATUS CameraInit(bool enable_soft_trigger);
    GX_STATUS CameraInit(char *serial_num, bool enable_soft_trigger);

    GX_STATUS SetExposureTime(double exposure_time);
    GX_STATUS SetFrameRate(double frame_rate);
    GX_STATUS SetWhileBalance();

    GX_STATUS CameraStreamOn();
    GX_STATUS CameraStreamOff();

    GX_STATUS CameraClose();

    GX_STATUS SendSoftTrigger();
    GX_STATUS GetLatestColorImg(cv::Mat &color_img);

    static bool is_lib_init_;

private:
    GX_STATUS CameraOpenDevice();  // Init as the first camera in device list
    GX_STATUS CameraOpenDevice(char *serial_num);  // Init with serial number

    GX_STATUS PrintCameraInfo();

    GX_STATUS CheckBasicProperties();
    GX_STATUS SetWorkingProperties();

    GX_STATUS CameraCloseDevice();

    ImageFormatConvertStatus ImageFormatConvert(PGX_FRAME_BUFFER &frame_buffer);

    void AllocImgBufferMem();
    void DeallocImgBufferMem();

    bool is_soft_trigger_mode_ = false;

    int64_t color_filter_ = GX_COLOR_FILTER_NONE;
    int64_t payload_size_ = 0;

    // PGX_FRAME_BUFFER frame_buffer_ = NULL;

    unsigned char* raw8_img_buff_ = NULL;
    unsigned char* rgb24_img_buff_ = NULL;

    GX_DEV_HANDLE device_handle_ = NULL;
};

enum class ImageFormatConvertStatus{
    kImageFormatConvertFail = -1,
    kImageFormatConvertSuccess = 0
};

// Used to handle driver status exception
class GxException : public std::exception {

public:
    GxException(GX_STATUS error_status) : error_status_(error_status){};
    virtual const char* what() const throw();

private:
    const char* GetErrorString(GX_STATUS emErrorStatus) const;

    GX_STATUS error_status_;

};

}

#endif
