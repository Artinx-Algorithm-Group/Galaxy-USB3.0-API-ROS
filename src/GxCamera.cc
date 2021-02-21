#include <iostream>

#include <opencv2/opencv.hpp>

#include "GxCamera.hpp"

using std::cout;
using std::cerr;
using std::endl;

using cv::Mat;
using cv::Size;

using GxCamera::Camera;
using GxCamera::GxException;
using GxCamera::ImageFormatConvertStatus;

namespace{
    const uint64_t kAcquireBufferNum = 5;  // Acquisition Buffer Quantity.
    const uint64_t kAcquireTransferSize = 64 * 1024;  // Size of data transfer block
    const uint64_t kAcquireTransferNumberURB = 64;  // Quantity of data transfer block
}

// Initialize static member
bool Camera::is_lib_init_ = false;

GX_STATUS Camera::CameraInit(bool enable_soft_trigger){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    this->is_soft_trigger_mode_  = enable_soft_trigger;

    // Init
    emStatus = this->CameraOpenDevice();
    if(emStatus != GX_STATUS_SUCCESS){
        cerr << "[GxCamera] Camera open device fail" << endl;
        return emStatus;
    }

    // Get info
    emStatus = this->PrintCameraInfo();
    if(emStatus != GX_STATUS_SUCCESS){
        cerr << "[GxCamera] Print camera info fail" << endl;
        return emStatus;
    }

    emStatus = this->CheckBasicProperties();
    if(emStatus != GX_STATUS_SUCCESS){
        cerr << "[GxCamera] Check basic properties fail" << endl;
        return emStatus;
    }

    emStatus = this->SetWorkingProperties();
    if(emStatus != GX_STATUS_SUCCESS){
        cerr << "[GxCamera] Set working properties fail" << endl;
        return emStatus;
    }

    this->AllocImgBufferMem();

    return emStatus;

}

GX_STATUS Camera::CameraInit(char *serial_num, bool enable_soft_trigger){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    this->is_soft_trigger_mode_  = enable_soft_trigger;

    // Init
    emStatus = this->CameraOpenDevice(serial_num);
    if(emStatus != GX_STATUS_SUCCESS){
        return emStatus;
    }

    // Get info
    emStatus = this->PrintCameraInfo();
    if(emStatus != GX_STATUS_SUCCESS){
        return emStatus;
    }

    emStatus = this->CheckBasicProperties();
    if(emStatus != GX_STATUS_SUCCESS){
        return emStatus;
    }

    emStatus = this->SetWorkingProperties();
    if(emStatus != GX_STATUS_SUCCESS){
        return emStatus;
    }

    this->AllocImgBufferMem();

    return emStatus;

}

GX_STATUS Camera::CameraOpenDevice() {

    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    if(this->is_lib_init_ == false){
        emStatus = GXInitLib();
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Lib init fail" << endl;
            return emStatus;
        }
        this->is_lib_init_ = true;
    }

    try{
        uint32_t device_num = 0;
        emStatus = GXUpdateDeviceList(&device_num, 1000);
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Update device list fail" << endl;
            throw GxException(emStatus);
        }

        if(device_num <= 0){
            cerr << "[GxCamera] No device found" << endl;
            emStatus = GX_STATUS_NOT_FOUND_DEVICE;
            throw GxException(emStatus);
        }

        emStatus = GXOpenDeviceByIndex(1, &this->device_handle_);
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Open device fail" << endl;
            throw GxException(emStatus);      
        }

    }catch(const GxException& e){
       cerr << e.what() << endl;
       GXCloseLib();
       cerr << "[GxCamera] Driver library closed" << endl;
       this->is_lib_init_ = false;
       return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::CameraOpenDevice(char *serial_num) {
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    if(this->is_lib_init_ == false){
        emStatus = GXInitLib();
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Galaxy driver library init fail" << endl;
            return emStatus;
        }
        this->is_lib_init_ = true;
    }

    try{
        uint32_t device_num = 0;
        emStatus = GXUpdateDeviceList(&device_num, 1000);
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Update device list fail, error info:" << endl;
            throw GxException(emStatus);
        }

        if(device_num <= 0){
            cerr << "[GxCamera] No device found" << endl;
            emStatus = GX_STATUS_NOT_FOUND_DEVICE;
            throw GxException(emStatus);
        }

        GX_OPEN_PARAM open_param;
        open_param.accessMode = GX_ACCESS_CONTROL;
        open_param.openMode = GX_OPEN_SN;
        open_param.pszContent = serial_num;
        emStatus = GXOpenDevice(&open_param,&this->device_handle_);
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Open device fail, error info" << endl;
            throw GxException(emStatus);      
        }

    }catch(const GxException& e){
       cerr << e.what() << endl;
       GXCloseLib();
       this->is_lib_init_ = false;
       cerr << "[GxCamera] Driver library closed" << endl;
       return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::SendSoftTrigger(){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    try{
        emStatus = GXSendCommand(this->device_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
        if (emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }
    }catch(const std::exception& e){
        cerr << "[GxCamera] Send soft trigger fail" << endl;
        cerr << e.what() << endl;
        return emStatus;
    }

    return emStatus;
    
}

GX_STATUS Camera::GetLatestColorImg(cv::Mat &color_img){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER frame_buffer[kAcquireBufferNum];
    try{
        uint32_t frame_count = 0;
        emStatus = GXDQAllBufs(this->device_handle_, frame_buffer, kAcquireBufferNum, &frame_count, 1000);
        if(emStatus == GX_STATUS_SUCCESS){
            int latest_idx = frame_count - 1;
            if(frame_buffer[latest_idx]->nStatus == GX_FRAME_STATUS_SUCCESS && frame_buffer[latest_idx] != NULL){
                this->ImageFormatConvert(frame_buffer[latest_idx]);
                color_img = Mat(Size(frame_buffer[latest_idx]->nWidth, frame_buffer[latest_idx]->nHeight), 
                                CV_8UC3, this->rgb24_img_buff_);
            }else{
                cerr << "[Galaxy] Get latest color image fail after DQAllBuf return success" << endl;
                return emStatus;
            }
        }else{
            throw GxException(emStatus);
        }

        emStatus = GXQAllBufs(this->device_handle_);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);  
        }
    }catch(const GxException& e){
        cerr << "[GxCamera] Get latest color image fail" << endl;
        cerr << e.what() << endl;
        return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::PrintCameraInfo(){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Get Device Info
    cout << "***********************************************" << endl;
    cout << "Device info:" << endl;

    //Get libary version
    //printf("<Libary Version : %s>\n", GXGetLibVersion());
    cout << "Library version: " << GXGetLibVersion() << endl;
    size_t nSize = 0;

    try{
        //Get string length of Vendor name
        emStatus = GXGetStringLength(this->device_handle_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);      
        }

        //Alloc memory for Vendor name
        char *pszVendorName = new char[nSize];
        //Get Vendor name
        emStatus = GXGetString(this->device_handle_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
        if (emStatus != GX_STATUS_SUCCESS){
            delete[] pszVendorName;
            pszVendorName = NULL;
            throw GxException(emStatus);
        }
        //printf("<Vendor Name : %s>\n", pszVendorName);
        cout << "Vendor name: " << pszVendorName << endl;

        //Release memory for Vendor name
        delete[] pszVendorName;
        pszVendorName = NULL;

        //Get string length of Model name
        emStatus = GXGetStringLength(this->device_handle_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);      
        }

        //Alloc memory for Model name
        char *pszModelName = new char[nSize];
        //Get Model name
        emStatus = GXGetString(this->device_handle_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
        if (emStatus != GX_STATUS_SUCCESS){
            delete[] pszModelName;
            pszModelName = NULL;
            throw GxException(emStatus);  
        }

        cout << "Model Name: " << pszModelName << endl;
        //Release memory for Model name
        delete[] pszModelName;
        pszModelName = NULL;

        //Get string length of Serial number
        emStatus = GXGetStringLength(this->device_handle_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        //Alloc memory for Serial number
        char *pszSerialNumber = new char[nSize];

        //Get Serial Number
        emStatus = GXGetString(this->device_handle_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
        if (emStatus != GX_STATUS_SUCCESS){
            delete[] pszSerialNumber;
            pszSerialNumber = NULL;
            throw GxException(emStatus);
        }

        printf("<Serial Number : %s>\n", pszSerialNumber);
        //Release memory for Serial number
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;

        //Get string length of Device version
        emStatus = GXGetStringLength(this->device_handle_, GX_STRING_DEVICE_VERSION, &nSize);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        char *pszDeviceVersion = new char[nSize];
        //Get Device Version
        emStatus = GXGetString(this->device_handle_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);

        if (emStatus != GX_STATUS_SUCCESS){
            delete[] pszDeviceVersion;
            pszDeviceVersion = NULL;
            throw GxException(emStatus);
        }

        //printf("<Device Version : %s>\n", pszDeviceVersion);
        cout << "Device version: " << pszDeviceVersion << endl;
        //Release memory for Device version
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        cout << "***********************************************" << endl;

    }catch(const GxException& e){
        cerr << e.what() << endl;;
        GXCloseDevice(this->device_handle_);
        this->device_handle_ = NULL;
        cerr << "[GxCamera] Device closed" << endl;
        GXCloseLib();
        this->is_lib_init_ = false;
        cerr << "[GxCamera] Library closed" << endl;
        return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::CheckBasicProperties(){
    bool is_color_cam;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        //Get the type of Bayer conversion. whether is a color camera.
        emStatus = GXIsImplemented(this->device_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &is_color_cam);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        // "Not color camera" is not considered as GxException
        if(!is_color_cam){
            cerr << "[GxCamera] Only support color cameras" << endl;
            GXCloseDevice(this->device_handle_);
            this->device_handle_ = NULL;
            cerr << "[GxCamera] Device closed" << endl;
            GXCloseLib();
            cerr << "[GxCamera] Library closed" << endl;
        }

        emStatus = GXGetEnum(this->device_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &this->color_filter_);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        emStatus = GXGetInt(this->device_handle_, GX_INT_PAYLOAD_SIZE, &this->payload_size_);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

    }catch(const GxException& e){
        cerr << e.what() << endl;
        GXCloseDevice(this->device_handle_);
        this->device_handle_ = NULL;
        cerr << "[GxCamera] Device closed" << endl;
        GXCloseLib();
        this->is_lib_init_ = false;
        cerr << "[GxCamera] Library closed" << endl;
        return emStatus;
    }
    
    return emStatus;
}

GX_STATUS Camera::SetWorkingProperties(){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        //Set acquisition mode
        emStatus = GXSetEnum(this->device_handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        //Set trigger mode
        if(this->is_soft_trigger_mode_){
            emStatus = GXSetEnum(this->device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        }else{
            emStatus = GXSetEnum(this->device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        }
        
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        //Set buffer quantity of acquisition queue
        uint64_t nBufferNum = kAcquireBufferNum;
        emStatus = GXSetAcqusitionBufferNumber(this->device_handle_, nBufferNum);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        bool bStreamTransferSize = false;
        emStatus = GXIsImplemented(this->device_handle_, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        if(bStreamTransferSize){
            //Set size of data transfer block
            emStatus = GXSetInt(this->device_handle_, GX_DS_INT_STREAM_TRANSFER_SIZE, kAcquireTransferSize);
            if(emStatus != GX_STATUS_SUCCESS){
                throw GxException(emStatus);
            }
        }

        bool bStreamTransferNumberUrb = false;
        emStatus = GXIsImplemented(this->device_handle_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        if(bStreamTransferNumberUrb){
            //Set qty. of data transfer block
            emStatus = GXSetInt(this->device_handle_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, kAcquireTransferNumberURB);
            if(emStatus != GX_STATUS_SUCCESS){
                throw GxException(emStatus);
            }
        }

        // Set Balance White Mode : Continuous as default
        emStatus = GXSetEnum(this->device_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        // Set auto exposure as default
        // emStatus = GXSetEnum(this->device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
        // if(emStatus != GX_STATUS_SUCCESS){
        //     throw GxException(emStatus);
        // }

    }catch(const GxException& e){
        cerr << e.what() << endl;
        GXCloseDevice(this->device_handle_);
        this->device_handle_ = NULL;
        cerr << "[GxCamera] Device closed" << endl;
        GXCloseLib();
        this->is_lib_init_ = false;
        cerr << "[GxCamera] Library closed" << endl;
        return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::CameraStreamOn(){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        //Device start acquisition
        emStatus = GXStreamOn(this->device_handle_);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }
    }catch(const GxException& e){
        cerr << e.what() << endl;
        this->DeallocImgBufferMem();
        cerr << "[GxCamera] Dealloc image buffer memory" << endl;
        GXCloseDevice(this->device_handle_);
        this->device_handle_ = NULL;
        cerr << "[GxCamera] Device closed" << endl;
        GXCloseLib();
        this->is_lib_init_ = false;
        cerr << "[GxCamera] Library closed" << endl;
        return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::CameraStreamOff(){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        //Device start acquisition
        emStatus = GXStreamOff(this->device_handle_);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }
    }catch(const GxException& e){
        cerr << e.what() << endl;
        this->DeallocImgBufferMem();
        cerr << "[GxCamera] Dealloc image buffer memory" << endl;
        GXCloseDevice(this->device_handle_);
        this->device_handle_ = NULL;
        cerr << "[GxCamera] Device closed" << endl;
        GXCloseLib();
        this->is_lib_init_ = false;
        cerr << "[GxCamera] Library closed" << endl;
        return emStatus;
    }

    return emStatus;
}

GX_STATUS Camera::CameraClose(){
    this->DeallocImgBufferMem();

    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    if(this->is_lib_init_){
        emStatus = this->CameraCloseDevice();
        if(emStatus != GX_STATUS_SUCCESS){
            cerr << "[GxCamera] Close camera fail" << endl;
            return emStatus;
        }
        this->is_lib_init_ = false;
    }

    return emStatus;
}

GX_STATUS Camera::CameraCloseDevice(){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        emStatus = GXCloseDevice(this->device_handle_);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        if(this->is_lib_init_){
            emStatus = GXCloseLib();
            if(emStatus != GX_STATUS_SUCCESS){
                throw GxException(emStatus);
            }
        }
    }catch(const GxException& e){
        cerr << e.what() << endl;
        return emStatus;
    }
    
    return emStatus;

}

GX_STATUS Camera::SetExposureTime(double exposure_time){
    //Set Expose Time
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        GX_FLOAT_RANGE shutter_range;
        emStatus = GXGetFloatRange(this->device_handle_, GX_FLOAT_EXPOSURE_TIME, &shutter_range);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        if(!(exposure_time >= shutter_range.dMin && exposure_time <= shutter_range.dMax)){
            cout << "[GxCamera] Invalid exposure time, data range: [" 
                 << shutter_range.dMin << ", " << shutter_range.dMax << "]" << endl
                 << "[GxCamera]Set exposure time fail" << endl;
            
            return emStatus;
        }

        emStatus = GXSetFloat(this->device_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_time);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        double current_exp;
        emStatus = GXGetFloat(this->device_handle_, GX_FLOAT_EXPOSURE_TIME, &current_exp);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        cout << "[GxCamera] Current camera exposure time: " << current_exp << endl;

    }catch(const GxException& e){
        cerr << e.what() << endl;
        cerr << "[GxCamera] Set exposure time fail" << endl;
    }

    return emStatus;
    
}

GX_STATUS Camera::SetFrameRate(double frame_rate){
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    try{
        //Enables the frame rate adjustment mode.
        emStatus = GXSetEnum(this->device_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                             GX_ACQUISITION_FRAME_RATE_MODE_ON);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        emStatus = GXSetFloat(this->device_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        double current_frame_rate;
        emStatus = GXGetFloat(this->device_handle_, GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE, &current_frame_rate);
        if(emStatus != GX_STATUS_SUCCESS){
            throw GxException(emStatus);
        }

        cout << "[GxCamera] Actual camera frame rate set to: " << current_frame_rate << endl;
        
    }catch(const GxException& e){
        cerr << e.what() << endl;
        cerr << "[GxCamera] Set frame rate fail" << endl;
    }

    return emStatus;
}

void Camera::AllocImgBufferMem(){

    if(payload_size_ != 0){
        this->raw8_img_buff_ = new unsigned char[this->payload_size_];
        this->rgb24_img_buff_ = new unsigned char[this->payload_size_ * 3];
    }else{
        cerr << "[GxCamera] Payload size not initiated when alloc img buffer" << endl;
        GXCloseDevice(this->device_handle_);
        this->device_handle_ = NULL;
        cerr << "[GxCamera] Device closed" << endl;
        GXCloseLib();
        this->is_lib_init_ = false;
        cerr << "[GxCamera] Library closed" << endl;
    }

}

void Camera::DeallocImgBufferMem(){
    //Release resources
    if (this->raw8_img_buff_ != NULL){
        delete[] this->raw8_img_buff_;
        this->raw8_img_buff_ = NULL;
    }
    if (this->rgb24_img_buff_ != NULL){
        delete[] this->rgb24_img_buff_;
        this->rgb24_img_buff_ = NULL;
    }
}

ImageFormatConvertStatus Camera::ImageFormatConvert(PGX_FRAME_BUFFER &frame_buffer){
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    switch (frame_buffer->nPixelFormat){
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image
            // emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, this->pRgb24ImgBuff, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
            //                   RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            DX_PIXEL_COLOR_FILTER ctype = BAYERRG;
            emDXStatus = DxRaw8toRGB24((unsigned char*)frame_buffer->pImgBuf, 
                                       this->rgb24_img_buff_, frame_buffer->nWidth, 
                                       frame_buffer->nHeight, RAW2RGB_NEIGHBOUR, ctype, false);
            if (emDXStatus != DX_OK){
                // printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                cerr << "DxRaw8toRGB24 Failed, Error Code: " << emDXStatus << endl;
                return ImageFormatConvertStatus::kImageFormatConvertFail;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)frame_buffer->pImgBuf, this->raw8_img_buff_, frame_buffer->nWidth, frame_buffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                // printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                cerr << "DxRaw16toRaw8 Failed, Error Code: " << emDXStatus << endl;
                return ImageFormatConvertStatus::kImageFormatConvertFail;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24((unsigned char*)this->raw8_img_buff_, this->rgb24_img_buff_, frame_buffer->nWidth, frame_buffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(this->color_filter_), false);
            if (emDXStatus != DX_OK)
            {
                // printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                cerr << "DxRaw16toRaw8 Failed, Error Code: " << emDXStatus << endl;
                return ImageFormatConvertStatus::kImageFormatConvertFail;
            }
            break;
        }
        default:
        {
            // printf("Error : PixelFormat of this camera is not supported\n");
            cerr << "Error : PixelFormat of this camera is not supported" << endl;
            return ImageFormatConvertStatus::kImageFormatConvertFail;
        }
    }
    return ImageFormatConvertStatus::kImageFormatConvertSuccess;
}

const char* GxException::GetErrorString(GX_STATUS error_status) const {
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    
    // Get length of error description
    emStatus = GXGetLastError(&error_status, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS){
        return "[LibraryInternalError] Error when calling GXGetLastError";
    }
    
    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL){
        return "[LibraryInternalError] Failed to allocate memory";
    }
    
    // Get error description
    emStatus = GXGetLastError(&error_status, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS){
        return "[LibraryInternalError] Error when calling GXGetLastError";
    }
    else{
        return error_info;
    }
}

const char* GxException::what() const throw(){
    return this->GetErrorString(this->error_status_);
}