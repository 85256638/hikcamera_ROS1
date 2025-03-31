#include <hik_camera_driver/camera.h>
#include <opencv2/imgproc/imgproc.hpp>  // For cvtColor if needed

namespace HIKCAMERA
{
    sensor_msgs::ImagePtr frame; // Temporary storage for the current frame
    pthread_mutex_t mutex;       // Mutex for storing frame
    bool frame_empty = true;     // Flag indicating if a new frame is available for publishing
    float exposure_time_set;     // Next exposure time to be set
    int exposure_auto;           // Flag indicating whether auto exposure is enabled

    // Define the global variables for dynamic frame rate and resolution
    float g_dynamic_framerate = 0.0;
    int g_resolution_width = 0;
    int g_resolution_height = 0;

    Hik_camera_base::Hik_camera_base(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        this->private_nh = private_nh;
        std::string cam_info_url;
        private_nh.param<std::string>("camera_name", camera_name, "camera");
        private_nh.param<std::string>("Camera/cam_info_url", cam_info_url, "");
        image_transport::ImageTransport it(nh);
        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, camera_name, cam_info_url));
        camera_pub = it.advertiseCamera(camera_name + "/image", 1);
        // exposure_sub = nh.subscribe<std_msgs::Float32>(camera_name + "/set_exposure", 10, boost::bind(&Hik_camera_base::exposure_callback, this, _1));
    }

    bool Hik_camera_base::set_params()
    {
        float frame_rate;
        bool trigger_mode;
        int trigger_line;
        int trigger_action;
        float trigger_delay;
        int Exposure;
        float Exposure_time;
        int ExposureTimeUp, ExposureTimeLow;
        int Gain_mode;
        float Gain_value;
        bool Digital_shift_mode;
        float Digital_shift;
        int brightneess;
        bool Gamma;
        float Gamma_value;
        int Gamma_selector;
        private_nh.param<float>("Camera/frame_rate", frame_rate, 10.0);
        private_nh.param<bool>("Camera/Trigger", trigger_mode, false);
        private_nh.param<int>("Camera/Tigger_line", trigger_line, 2);
        private_nh.param<int>("Camera/Trigger_action", trigger_action, 0);
        private_nh.param<float>("Camera/Trigger_delay", trigger_delay, 0.0);
        private_nh.param<int>("Camera/Exposure", Exposure, 2);
        private_nh.param<float>("Camera/Exposure_time", Exposure_time, 10000.0);
        private_nh.param<int>("Camera/ExposureTimeUp", ExposureTimeUp, 6000);
        private_nh.param<int>("Camera/ExposureTimeLow", ExposureTimeLow, 100);
        private_nh.param<int>("Camera/Gain_mode", Gain_mode, 2);
        private_nh.param<float>("Camera/Gain_value", Gain_value, 0.0);
        private_nh.param<bool>("Camera/Digital_shift_mode", Digital_shift_mode, true);
        private_nh.param<float>("Camera/Digital_shift", Digital_shift, 3.0);
        private_nh.param<int>("Camera/brightneess", brightneess, 100);
        private_nh.param<bool>("Camera/GammaEnable", Gamma, false);
        private_nh.param<float>("Camera/Gamma_value", Gamma_value, 1.0);
        private_nh.param<int>("Camera/Gamma_selector", Gamma_selector, 1);
        private_nh.param<bool>("Camera/Exposure_control", exposure_control, false);
        
        // Offset parameters
    int offsetX, offsetY;
    private_nh.param<int>("Camera/offsetX", offsetX, 0);   // Default offsetX = 0
    private_nh.param<int>("Camera/offsetY", offsetY, 0);   // Default offsetY = 0
    ROS_INFO_STREAM("Setting offsets to: offsetX = " << offsetX << ", offsetY = " << offsetY);

    // Set offsetX and offsetY using the camera SDK
    nRet = MV_CC_SetIntValue(m_handle, "OffsetX", offsetX);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Failed to set OffsetX! nRet [%x]", nRet);
        return false;
    }
    nRet = MV_CC_SetIntValue(m_handle, "OffsetY", offsetY);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Failed to set OffsetY! nRet [%x]", nRet);
        return false;
    }

        // Add resolution parameters:
        int width, height;
        private_nh.param<int>("Camera/width", width, 1280);   // Default width = 1280
        private_nh.param<int>("Camera/height", height, 1024);  // Default height = 1024
        ROS_INFO_STREAM("Setting resolution to: " << width << "x" << height);
        nRet = MV_CC_SetIntValue(m_handle, "Width", width);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Failed to set width! nRet [%x]", nRet);
            return false;
        }
        nRet = MV_CC_SetIntValue(m_handle, "Height", height);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Failed to set height! nRet [%x]", nRet);
            return false;
        }

        exposure_time_set = Exposure_time;
        exposure_auto = Exposure;
        exposure_time_low = ExposureTimeLow;
        exposure_time_up = ExposureTimeUp;
        light_set = brightneess;

        setEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        ROS_INFO_STREAM("AcquisitionMode set to Continuous.");
        // Set trigger parameters if enabled
        if (trigger_mode)
        {
            setEnumValue("TriggerMode", MV_TRIGGER_MODE_ON);
            ROS_INFO_STREAM("TriggerMode set to ON");
            if (trigger_line >= 0 && trigger_line <= 8 && trigger_line != 5 && trigger_line != 6)
            {
                ROS_INFO_STREAM("TriggerSource set to " << trigger_line);
                setEnumValue("TriggerSource", trigger_line);
                if (trigger_action >= 0 && trigger_action <= 3)
                {
                    ROS_INFO_STREAM("TriggerActivation set to " << trigger_action);
                    setEnumValue("TriggerActivation", trigger_action);
                }
                else
                {
                    ROS_WARN_STREAM("Not Exist Trigger Action: " << trigger_action);
                }
                setFloatValue("TriggerDelay", trigger_delay);
                ROS_INFO_STREAM("TriggerDelay set to " << trigger_delay << "us.");
            }
            else
            {
                ROS_WARN_STREAM("Not Exist Trigger Source: " << trigger_line);
            }
        }
        else
        {
            ROS_INFO_STREAM("TriggerMode set to OFF");
            setEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
        }
        // Set frame rate
        setFrameRate(frame_rate);
        ROS_INFO_STREAM("Frame Rate set to " << frame_rate << " fps");
        // Set exposure
        if (Exposure == 0)
        {
            setEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
            setFloatValue("ExposureTime", Exposure_time);
            ROS_INFO_STREAM("ExposureAuto set to OFF");
            ROS_INFO_STREAM("ExposureTime set to " << Exposure_time << "us");
        }
        else if (Exposure > 0 && Exposure <= 2)
        {
            setEnumValue("ExposureAuto", Exposure);
            setIntValue("AutoExposureTimeLowerLimit", ExposureTimeLow);
            setIntValue("AutoExposureTimeUpperLimit", ExposureTimeUp);
            ROS_INFO_STREAM("ExposureAuto set to " << Exposure);
            ROS_INFO_STREAM("AutoExposureTimeLowerLimit set to " << ExposureTimeLow << "us");
            ROS_INFO_STREAM("AutoExposureTimeUpperLimit set to " << ExposureTimeUp << "us");
        }
        else
        {
            ROS_ERROR_STREAM("Not Exist Exposure Mode: " << Exposure);
            return false;
        }
        // Set Gain
        if (Gain_mode == 0)
        {
            setEnumValue("GainAuto", MV_GAIN_MODE_OFF);
            setFloatValue("Gain", Gain_value);
            ROS_INFO_STREAM("GainAuto set to OFF");
            ROS_INFO_STREAM("Gain set to " << Gain_value << "dB");
        }
        else if (Gain_mode > 0 && Gain_mode <= 2)
        {
            setEnumValue("GainAuto", Gain_mode);
            ROS_INFO_STREAM("GainAuto set to " << Gain_mode);
        }
        else
        {
            ROS_ERROR_STREAM("Not Exist Gain Mode: " << Gain_mode);
            return false;
        }
        // Set White Balance
        setEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
        ROS_INFO_STREAM("BalanceWhiteAuto set to Continuous");
        // Set Pixel Format (adjust according to camera support)
        setEnumValue("PixelFormat", PixelType_Gvsp_BayerGB10);
        // Set Brightness
        setIntValue("Brightness", brightneess);
        ROS_INFO_STREAM("Brightness set to " << brightneess);
        // Set Digital Shift
        if (Digital_shift_mode)
        {
            setBoolValue("DigitalShiftEnable", true);
            setFloatValue("DigitalShift", Digital_shift);
            ROS_INFO_STREAM("DigitalShiftEnable set to ON");
            ROS_INFO_STREAM("DigitalShift set to " << Digital_shift);
        }
        else
        {
            setBoolValue("DigitalShiftEnable", false);
            ROS_INFO_STREAM("DigitalShiftEnable set to OFF");
        }
        return true;
    }

    bool Hik_camera_base::set_camera(MV_CC_DEVICE_INFO &camera)
    {
        m_stDevInfo = camera;
        nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Error: CreateHandle fail! nRet [%x]\n", nRet);
            return false;
        }
        unsigned int nAccessMode = MV_ACCESS_Exclusive;
        unsigned short nSwitchoverKey = 0;
        nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
        if (MV_OK != nRet)
        {
            ROS_ERROR("error: OpenDevice fail! nRet [%x]\n", nRet);
            return false;
        }
        if (m_stDevInfo.nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(m_handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(m_handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    ROS_WARN("Warning: Set Packet Size fail! nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                ROS_WARN("Warning: Get Packet Size fail! nRet [0x%x]!\n", nPacketSize);
            }
        }
        return true;
    }

    bool Hik_camera_base::setFrameRate(float frame_rate)
    {
        nRet = MV_CC_SetFrameRate(m_handle, frame_rate);
        if (MV_OK != nRet)
        {
            ROS_ERROR("frame rate set fail! nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setEnumValue(std::string name, unsigned int value)
    {
        nRet = MV_CC_SetEnumValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setBoolValue(std::string name, bool value)
    {
        nRet = MV_CC_SetBoolValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setFloatValue(std::string name, float value)
    {
        nRet = MV_CC_SetFloatValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setStringValue(std::string name, std::string value)
    {
        nRet = MV_CC_SetStringValue(m_handle, name.c_str(), value.c_str());
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setIntValue(std::string name, unsigned int value)
    {
        nRet = MV_CC_SetIntValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setCommandValue(std::string name)
    {
        nRet = MV_CC_SetCommandValue(m_handle, name.c_str());
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::getEnumValue(std::string name, MVCC_ENUMVALUE &value)
    {
        nRet = MV_CC_GetEnumValue(m_handle, name.c_str(), &value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::getBoolValue(std::string name, bool &value)
    {
        nRet = MV_CC_GetBoolValue(m_handle, name.c_str(), &value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::getFloatValue(std::string name, float &value)
    {
        MVCC_FLOATVALUE temp_value;
        nRet = MV_CC_GetFloatValue(m_handle, name.c_str(), &temp_value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = temp_value.fCurValue;
        return true;
    }

    bool Hik_camera_base::getStringValue(std::string name, std::string &value)
    {
        MVCC_STRINGVALUE stStringValue = {0};
        nRet = MV_CC_GetStringValue(m_handle, name.c_str(), &stStringValue);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = stStringValue.chCurValue;
        return true;
    }

    bool Hik_camera_base::getIntValue(std::string name, unsigned int &value)
    {
        MVCC_INTVALUE stHeight = {0};
        nRet = MV_CC_GetIntValue(m_handle, name.c_str(), &stHeight);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = stHeight.nCurValue;
        return true;
    }

    bool Hik_camera_base::ImageStream()
    {
        nRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            ROS_ERROR("pthread create failed\n");
            return false;
        }
        nRet = pthread_create(&nThreadID, NULL, WorkThread, m_handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("thread create failed.ret = %d\n", nRet);
            return false;
        }
        return true;
    }

    void Hik_camera_base::stopStream()
    {
        nRet = MV_CC_StopGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return;
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        nRet = MV_CC_CloseDevice(m_handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return;
        }
        printf("MV_CC_CloseDevice succeed.\n");
        nRet = MV_CC_DestroyHandle(m_handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return;
        }
        printf("MV_CC_DestroyHandle succeed.\n");
    }

    bool Hik_camera_base::changeExposureTime(float value)
    {
        nRet = MV_CC_StopGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        setFloatValue("ExposureTime", value);
        nRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    void Hik_camera_base::exposure_callback(const std_msgs::Float32ConstPtr msg)
    {
        float exposure = msg->data;
        float get_exposure;
        getFloatValue("ExposureTime", get_exposure);
        if (get_exposure != exposure)
        {
            // changeExposureTime(exposure);
            exposure_time_set = exposure;
        }
    }

    // Updated WorkThread: dynamically calculates frame rate and resolution once per second,
    // and stores these values in global variables for later printing in the main loop.
    void *Hik_camera_base::WorkThread(void *p_handle)
    {
        int nRet = MV_OK;
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(p_handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return NULL;
        }
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(stParam.nCurValue * 3);
        if (NULL == pData)
            return NULL;
        unsigned int nDataSize = stParam.nCurValue;

        // Variables for dynamic frame rate calculation
        int frame_count = 0;
        ros::Time last_time = ros::Time::now();

        while (ros::ok())
        {
            if (exposure_auto == 0)
            {
                nRet = MV_CC_SetExposureTime(p_handle, exposure_time_set);
                if (MV_OK != nRet)
                {
                    ROS_WARN("Exposure time set failed! nRet [%x]\n", nRet);
                }
            }
            nRet = MV_CC_GetOneFrameTimeout(p_handle, pData, nDataSize, &stImageInfo, 50);
            if (nRet == MV_OK)
            {
                ros::Time rcv_time = ros::Time::now();

                stConvertParam.nWidth = stImageInfo.nWidth;
                stConvertParam.nHeight = stImageInfo.nHeight;
                stConvertParam.pSrcData = pData;
                stConvertParam.nSrcDataLen = nDataSize;
                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                stConvertParam.pDstBuffer = m_pBufForSaveImage;
                stConvertParam.nDstBufferSize = nDataSize * 3;
                stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
                MV_CC_ConvertPixelTypeEx(p_handle, &stConvertParam);

                cv::Mat srcImage(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
                msg->header.stamp = rcv_time;
                
                pthread_mutex_lock(&mutex);
                frame_empty = false;
                frame = msg;
                pthread_mutex_unlock(&mutex);

                // Increment frame counter and calculate dynamic frame rate every second
                frame_count++;
                ros::Time current_time = ros::Time::now();
                double elapsed = (current_time - last_time).toSec();
                if (elapsed >= 1.0) // Once per second
                {
                    double dynamic_framerate = frame_count / elapsed;
                    g_dynamic_framerate = dynamic_framerate;
                    g_resolution_width = stImageInfo.nWidth;
                    g_resolution_height = stImageInfo.nHeight;
                    frame_count = 0;
                    last_time = current_time;
                }
            }
        }
        if (pData)
        {
            free(pData);
            pData = NULL;
        }
        if (m_pBufForSaveImage)
        {
            free(m_pBufForSaveImage);
            m_pBufForSaveImage = NULL;
        }
        return NULL;
    }

    void Hik_camera_base::ImagePub()
    {
        pthread_mutex_lock(&mutex);
        if (frame_empty == false)
        {
            frame->header.frame_id = camera_name;
            sensor_msgs::CameraInfoPtr ci_(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ci_->header.frame_id = frame->header.frame_id;
            ci_->header.stamp = frame->header.stamp;
            camera_pub.publish(*frame, *ci_);
            frame_empty = true;

            if (exposure_control)
            {
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
                cv::Mat temp_img = cv_ptr->image;
                cv::Mat imgGray;
                cv::cvtColor(temp_img, imgGray, CV_BGR2GRAY);
                cv::Scalar grayScalar = cv::mean(imgGray);
                float imgGrayLight = grayScalar.val[0];
                if (imgGrayLight < light_set - 10 && exposure_time_set * scale < exposure_time_up)
                {
                    exposure_time_set *= scale;
                }
                else if (imgGrayLight > light_set + 10 && exposure_time_set / scale > exposure_time_low)
                {
                    exposure_time_set /= scale;
                }
            }
        }
        pthread_mutex_unlock(&mutex);
    }

} // namespace HIKCAMERA

