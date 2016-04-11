#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <polled_camera/publication_server.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <boost/thread.hpp>

#include <netusb_camera_driver/NetUSBCameraConfig.h>
#include <netusb_camera_driver/netusb_defs.h>
#include <netusb_camera_driver/netusb_utils.hpp>

#include <NETUSBCAM_API.h>

namespace netusb_camera
{

class NetUSBNodelet;

int GetImageCallback(void *buffer, unsigned int buffersize, void *context);

class NetUSBNodelet : public nodelet::Nodelet
{

public:

    void GetImage(void * buffer, unsigned int buffersize) {
        publishImage(buffer, buffersize, ros::Time::now());
    }

    virtual ~NetUSBNodelet()
    {
        stop();
        
        trigger_sub_.shutdown();
        poll_srv_.shutdown();
        image_publisher_.shutdown();

        // close camera
        int result = NETUSBCAM_Close(camera_id_);
        if(result!=0){
            NODELET_ERROR("Error: Close; Result = %d", result);
        }

        NODELET_WARN("Unloaded Net USB camera with id %d", camera_id_);
    }

    NetUSBNodelet() : 
        param_info_(PARAM_MAX+1),
        param_mask_(PARAM_MAX+1, false)
    {
    }

private:

    int camera_id_;
    std::string serial_;
    std::string camera_name_;
    bool camera_opened_;
    
    int width_;
    int height_;
    int color_mode_;
    int video_mode_;
    
    bool video_mode_mask_[10];
    
    std::vector<PARAM_PROPERTY> param_info_;
    std::vector<bool> param_mask_;
    
    ros::Timer update_timer_;

    image_transport::CameraPublisher image_publisher_;
    polled_camera::PublicationServer poll_srv_;
    ros::ServiceServer               set_camera_info_srv_;
    ros::Subscriber                  trigger_sub_;

    /** camera calibration information */
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    bool calibration_matches_;            // CameraInfo matches video mode

    sensor_msgs::Image img_;
    sensor_msgs::CameraInfo cam_info_;

    std::string   frame_id_;

    std::string   trig_timestamp_topic_;
    ros::Time     trig_time_;

    // Dynamic reconfigure parameters
    double        update_rate_;
    int           trigger_mode_;

    // Dynamic Reconfigure
    netusb_camera_driver::NetUSBCameraConfig last_config_;
    boost::recursive_mutex config_mutex_;
    typedef dynamic_reconfigure::Server<netusb_camera_driver::NetUSBCameraConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    // State updater
    enum CameraState
    {
        OPENING,
        CAMERA_NOT_FOUND,
        FORMAT_ERROR,
        ERROR,
        OK
    } camera_state_;
    
    std::string state_info_;
    std::string intrinsics_;
    diagnostic_updater::Updater updater;


    virtual void onInit()
    {
        int camera_cnt;
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& pn = getPrivateNodeHandle();

        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh));
        NODELET_INFO("Initializing NET SDK");
        camera_cnt = NETUSBCAM_Init(); // look for ICubes
        if(camera_cnt == 0) 
        {
            NODELET_ERROR("No device\n");
            return; 
        }

        //! Retrieve parameters from server
        NODELET_INFO("namespace: %s", pn.getNamespace().c_str());
        pn.param<std::string>("frame_id", frame_id_, "/camera_optical_frame");
        NODELET_INFO("Loaded param frame_id: %s", frame_id_.c_str());

        pn.param<int>("camera_id", camera_id_, 0);
        NODELET_INFO("Loaded camera id: %d", camera_id_);

        std::string color_mode;
        pn.param<std::string>("color_mode", color_mode, "rgb24");
        if (color_mode == "rgb24") color_mode_ = CALLBACK_RGB; else
        if (color_mode == "raw8") color_mode_ = CALLBACK_RAW; else
            NODELET_ERROR("Wrong color mode [%s] specified!", color_mode.c_str());
        NODELET_INFO("Loaded color mode: %s", color_mode.c_str());
        
        pn.param<int>("video_mode", video_mode_, 5);
        NODELET_INFO("Loaded video mode: %d", video_mode_);

        // Open camera
        openCamera();
        
        // Setup updater
        updater.add(getName().c_str(), this, &NetUSBNodelet::getCurrentState);
        NODELET_INFO("updated state");
        // Setup periodic callback to get new data from the camera
        update_timer_ = nh.createTimer(ros::Rate(10).expectedCycleTime(), &NetUSBNodelet::updateCallback, this, false ,false);
        update_timer_.stop();
        NODELET_INFO("created update timer");

        // Advertise topics
        ros::NodeHandle image_nh(nh);
        image_transport::ImageTransport image_it(image_nh);
        image_publisher_ = image_it.advertiseCamera("image_raw", 1);
        poll_srv_ = polled_camera::advertise(nh, "request_image", &NetUSBNodelet::pollCallback, this);
        trigger_sub_ = pn.subscribe(trig_timestamp_topic_, 1, &NetUSBNodelet::syncInCallback, this);

        // Setup dynamic reconfigure server
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pn));
        ReconfigureServer::CallbackType f = boost::bind(&NetUSBNodelet::reconfigureCallback, this, _1, _2);
        reconfigure_server_->setCallback(f);
    }

    void openCamera()
    {
        int result;

        boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex_);
        camera_state_ = OPENING;
        NODELET_INFO("Trying to open camera with id %d", camera_id_);

        result = NETUSBCAM_Open(camera_id_);  // open camera 
        if(result != 0)
        {
            NODELET_ERROR("Error: Open; Result = %d", result);
            return; 
        }
        
        
        result = NETUSBCAM_SetCamParameter(camera_id_, REG_CALLBACK_BR_FRAMES, 1);

        char name[20];          // get camera model name
        result = NETUSBCAM_GetName(camera_id_, name, 20);
        if (result!=0) 
        {
            NODELET_ERROR("Error: GetName; Result = %d\n", result);
            return; 
        }
        camera_name_ = name;

        char serial[20];          // get camera serial
        result = NETUSBCAM_GetSerialNum(camera_id_, serial, 20);
        if (result!=0) 
        {
            NODELET_ERROR("Error: GetSerialNum; Result = %d\n", result);
            return; 
        }
        serial_ = serial;
        
        for (int i = 0; i < 10; ++i) {
            video_mode_mask_[i] = false;
        }
        unsigned int mode_len;
        unsigned int mode_list[10];
        result = NETUSBCAM_GetModeList(camera_id_, &mode_len, mode_list);
        if (result!=0) 
        {
            NODELET_ERROR("Error: GetModeList; Result = %d\n", result);
            return; 
        }
        NODELET_INFO("Available video modes:");
        for (int i = 0; i < mode_len; ++i) {
            NODELET_INFO("%d", mode_list[i]);
            video_mode_mask_[mode_list[i]] = true;
        }
        
        updater.setHardwareIDf("%s (%s)", serial, name);

        NODELET_INFO("Started Net USB camera - id: %d; serial: %s; model: %s", camera_id_, serial, name);

        
        readAvailableParams();
        printParams();
        
        start();

    }

    std::string getAvailableCameras()
    {
        return "Not implemented yet";
    }


    void start()
    {
        NODELET_INFO("START");
        int result;
        try
        {
            result = NETUSBCAM_SetMode(camera_id_, video_mode_);
            if ( result != 0 ){
                NODELET_ERROR("Error: SetMode; Result = %d", result);
                return;
            }
            
            result = NETUSBCAM_GetSize(camera_id_, &width_, &height_);
            if ( result != 0 ){
                NODELET_ERROR("Error: GetSize; Result = %d", result);
                return;
            }
            
            
            // set the callback to get the frame buffer
            result = NETUSBCAM_SetCallback(camera_id_, color_mode_, &GetImageCallback, (void*)this);
            if(result!=0)
            {
                NODELET_ERROR("Error: SetCallback; Result = %d [%s]", result, err2str(result).c_str());
                return; 
            }             

            // start streaming of camera
            result = NETUSBCAM_Start(camera_id_);
            if(result!=0)
            {
                NODELET_ERROR("Error: Start; Result = %d\n", result);
                return;
            }
        }
        catch(std::exception &e)
        {
            camera_state_ = CAMERA_NOT_FOUND;
            state_info_ = e.what();
        }
        //update_timer_.start();
    }

    void stop()
    {
        NODELET_INFO("STOP");
        // stop streaming of camera
        int result = NETUSBCAM_Stop(camera_id_);
        if(result != 0)
        {
            NODELET_ERROR("Error: Stop; Result = %d\n", result);
            return; 
        }

        NETUSBCAM_SetCallback(camera_id_, CALLBACK_RGB, NULL, NULL);
        update_timer_.stop();
    }
    
    void readAvailableParams() {
        for (int i = 0; i < PARAM_MAX; ++i) {
            int result = NETUSBCAM_GetCamParameterRange(camera_id_, i, &param_info_[i]);
            if (result != 0) 
            {
                param_info_[i].bEnabled = false;
                //NODELET_INFO("Param %3d %-25s: %d %s", i, param2str(i).c_str(), result, err2str(result).c_str());
            } 
            else 
            {
                param_mask_[i] = true;
                //NODELET_INFO("Param %3d %-25s: OK", i, param2str(i).c_str());
            }
        }
    }

    void printParams() { 
        for (int i = 0; i < PARAM_MAX; ++i) {
            if (param_mask_[i])
            {        
                NODELET_INFO("Param %3d %-25s: %c%c%c [%u, %u, %lu]",
                                i, 
                                param2str(i).c_str(), 
                                chariftrue(param_info_[i].bEnabled, 'E'),
                                chariftrue(param_info_[i].bAuto, 'A'),
                                chariftrue(param_info_[i].bOnePush, 'O'),
                                param_info_[i].nMin, 
                                param_info_[i].nDef,
                                param_info_[i].nMax
                            );
            }
        }
    }

    void publishImage(void * buffer, unsigned int buffersize, ros::Time time)
    {
        camera_state_ = OK;
        state_info_ = "Camera operating normally";
        if (image_publisher_.getNumSubscribers() > 0)
        {

            if (processFrame(buffer, buffersize, img_, cam_info_))
            {
                image_publisher_.publish(img_, cam_info_, time);
            }
            else
            {
                camera_state_ = FORMAT_ERROR;
                state_info_ = "Unable to process frame";
            }
        }
        updater.update();
    }

    void updateCallback(const ros::TimerEvent &event)
    {
        /*
        // Download the most recent data from the device
        camera_state_ = OK;
        state_info_ = "Camera operating normally";
        if(image_publisher_.getNumSubscribers() > 0)
        {
            boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
            try
            {
                tPvFrame* frame = NULL;
                frame = camera_->grab(1000);
                publishImage(frame, event.current_real);
            }
            catch(std::exception &e)
            {
                camera_state_ = ERROR;
                state_info_ = e.what();
                NODELET_ERROR("Unable to read from camera: %s", e.what());
                ++frames_dropped_total_;
                frames_dropped_acc_.add(1);
                updater.update();
                return;
            }
        }*/
        updater.update();
    }

    void pollCallback(polled_camera::GetPolledImage::Request& req,
                      polled_camera::GetPolledImage::Response& rsp,
                      sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
    {
       /* if (trigger_mode_ != prosilica::Software)
        {
            rsp.success = false;
            rsp.status_message = "Camera is not in software triggered mode";
            return;
        }

        last_config_.binning_x = req.binning_x;
        last_config_.binning_y = req.binning_y;
        last_config_.x_offset = req.roi.x_offset;
        last_config_.y_offset = req.roi.y_offset;
        last_config_.height   = req.roi.height;
        last_config_.width    = req.roi.width;

        reconfigureCallback(last_config_, driver_base::SensorLevels::RECONFIGURE_RUNNING);

        try
        {
            tPvFrame* frame = NULL;
            frame = camera_->grab(req.timeout.toSec()*100);
            if (processFrame(frame, image, info))
            {
                image.header.stamp = info.header.stamp =rsp.stamp = ros::Time::now();
                rsp.status_message = "Success";
                rsp.success = true;
            }
            else
            {
                rsp.success = false;
                rsp.status_message = "Failed to process image";
                return;
            }
        }
        catch(std::exception &e)
        {
            rsp.success = false;
            std::stringstream err;
            err<< "Failed to grab frame: "<<e.what();
            rsp.status_message = err.str();
            return;
        }*/
    }

    void syncInCallback (const std_msgs::HeaderConstPtr& msg)
    {
        /*if (trigger_mode_ != prosilica::Software)
        {
            camera_state_ = ERROR;
            state_info_ = "Can not sync from topic trigger unless in Software Trigger mode";
            NODELET_ERROR_ONCE("%s", state_info_.c_str());
            return;
        }
        ros::TimerEvent e;
        e.current_real = msg->stamp;
        updateCallback(e);*/
    }

    bool processFrame(void * buffer, unsigned int buffersize, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
    {
        if (buffersize > 0)
            return frameToImage(buffer, buffersize, img);
        else
        {
            NODELET_ERROR("Frame error");
            return false;
        }
        
    }

    bool frameToImage(void * buffer, unsigned int buffersize, sensor_msgs::Image &image)
    {
        std::string encoding;
        if (color_mode_ == CALLBACK_RAW) encoding = "bayer_bggr8";
        if (color_mode_ == CALLBACK_RGB) encoding = "rgb8"; 
        
        if(buffersize == 0)
            return false;
          
        uint32_t step = buffersize / height_;

        return sensor_msgs::fillImage(image, encoding, height_, width_, step, buffer);
    }

    bool setCameraInfo(sensor_msgs::SetCameraInfoRequest &req, sensor_msgs::SetCameraInfoResponse &rsp)
    {
        NODELET_INFO("New camera info received");
        /*sensor_msgs::CameraInfo &info = req.camera_info;

        // Sanity check: the image dimensions should match the max resolution of the sensor.
        if (info.width != sensor_width_ || info.height != sensor_height_)
        {
            rsp.success = false;
            rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                                "setting, camera running at resolution %ix%i.")
                                                 % info.width % info.height % sensor_width_ % sensor_height_).str();
            NODELET_ERROR("%s", rsp.status_message.c_str());
            return true;
        }

        stop();

        std::string cam_name = "prosilica";
        cam_name += hw_id_;
        std::stringstream ini_stream;
        if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, cam_name, info))
        {
            rsp.status_message = "Error formatting camera_info for storage.";
            rsp.success = false;
        }
        else
        {
            std::string ini = ini_stream.str();
            if (ini.size() > prosilica::Camera::USER_MEMORY_SIZE)
            {
                rsp.success = false;
                rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
            }
            else
            {
                try
                {
                    camera_->writeUserMemory(ini.c_str(), ini.size());
                    cam_info_ = info;
                    rsp.success = true;
                }
                catch (prosilica::ProsilicaException &e)
                {
                    rsp.success = false;
                    rsp.status_message = e.what();
                }
            }
        }
        if (!rsp.success)
        NODELET_ERROR("%s", rsp.status_message.c_str());

        start();

        return true;*/
        
        return false;
    }

    bool setParam(int param, int & value) {
        int result = NETUSBCAM_SetCamParameter(camera_id_, param, value);
        if(result != 0)
        {
            NODELET_ERROR("Setting parameter %s=%d: %s. Proper range [%u, %lu]", param2str(param).c_str(), value, err2str(result).c_str(), param_info_[param].nMin, param_info_[param].nMax);
            long unsigned int tmp;
            NETUSBCAM_GetCamParameter(camera_id_, param, &tmp);
            value = tmp;
            return false;
        } 
        
        return true;
    }
    
    bool setParam(int param, bool & value) {
        int result = NETUSBCAM_SetCamParameter(camera_id_, param, value ? 1 : 0);
        if(result != 0)
        {
            NODELET_ERROR("Setting parameter %s=%s: %s. Proper range [%u, %lu]", param2str(param).c_str(), value ? "TRUE" : "FALSE", err2str(result).c_str(), param_info_[param].nMin, param_info_[param].nMax);
            long unsigned int tmp;
            NETUSBCAM_GetCamParameter(camera_id_, param, &tmp);
            value = (tmp == 1);
            return false;
        } 
        
        return true;
    }
    
    bool setExposure(double & value) {
        float f_exp = value * 1000;
        int result = NETUSBCAM_SetExposure(camera_id_, f_exp);
        if(result != 0)
        {
            PARAM_PROPERTY_f exp_range;
            NETUSBCAM_GetExposureRange(camera_id_, &exp_range);
            NODELET_ERROR("Setting EXPOSURE=%f: %s. Proper range [%.3f, %.3f]", value, err2str(result).c_str(), exp_range.nMin * 0.001, exp_range.nMax * 0.001);
            NETUSBCAM_GetExposure(camera_id_, &f_exp);
            value = f_exp * 0.001;
            return false;
        } 
        
        return true;
    }

    void reconfigureCallback(netusb_camera_driver::NetUSBCameraConfig &config, uint32_t level)
    {
        NODELET_INFO("Reconfigure request received");
        int result;
        
        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
            stop();

        if (config.color_mode == "rgb24") color_mode_ = CALLBACK_RGB; else
        if (config.color_mode == "raw8") color_mode_ = CALLBACK_RAW; 
        
        if (video_mode_mask_[config.video_mode])
        {
            video_mode_ = config.video_mode;
        }
        else
        {
            NODELET_ERROR("Unsupported video mode [%d]. Ignoring.", config.video_mode);
            config.video_mode = video_mode_;
        }

        setParam(REG_PLL, config.pll);
        setParam(REG_GAIN, config.gain);
        setExposure(config.exposure);
        
        setParam(REG_DEFECT_COR, config.defect_corretction);
        setParam(REG_INVERT_PIXEL, config.invert_pixel);
        
  /*      //! Trigger mode
        if (config.trigger_mode == "streaming")
        {
            trigger_mode_ = prosilica::Freerun;
            update_rate_ = 1.; // make sure we get _something_
        }
        else if (config.trigger_mode == "syncin1")
        {
            trigger_mode_ = prosilica::SyncIn1;
            update_rate_ = config.trig_rate;
        }
        else if (config.trigger_mode == "syncin2")
        {
            trigger_mode_ = prosilica::SyncIn2;
            update_rate_ = config.trig_rate;
        }
        else if (config.trigger_mode == "fixedrate")
        {
            trigger_mode_ = prosilica::FixedRate;
            update_rate_ = config.trig_rate;
        }
        else if (config.trigger_mode == "software")
        {
            trigger_mode_ = prosilica::Software;
            update_rate_ = config.trig_rate;
        }

        else if (config.trigger_mode == "polled")
        {
            trigger_mode_ = prosilica::Software;
            update_rate_ = 0;
        }
        else if (config.trigger_mode == "triggered")
        {
            trigger_mode_ = prosilica::Software;
            update_rate_ = 0;
        }
        else
        {
            NODELET_ERROR("Invalid trigger mode '%s' in reconfigure request", config.trigger_mode.c_str());
        }

        if(config.trig_timestamp_topic != last_config_.trig_timestamp_topic)
        {
            trigger_sub_.shutdown();
            trig_timestamp_topic_ = config.trig_timestamp_topic;
        }

        if(!trigger_sub_ && config.trigger_mode == "triggered")
        {
            trigger_sub_ = ros::NodeHandle().subscribe(trig_timestamp_topic_, 1, &NetUSBNodelet::syncInCallback, this);
        }

*/
        // restart camera
        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
            start();
    }

    void getCurrentState(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        stat.add("Serial", serial_);
        stat.add("Info", state_info_);

        switch (camera_state_)
        {
            case OPENING:
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Opening camera");
                break;
            case OK:
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera operating normally");
                break;
            case CAMERA_NOT_FOUND:
                stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Can not find camera %d", camera_id_ );
                break;
            case FORMAT_ERROR:
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Problem retrieving frame");
                break;
            case ERROR:
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera has encountered an error");
                break;
            default:
                break;
        }
    }
};


int GetImageCallback(void *buffer, unsigned int buffersize, void *context)
{
    ((NetUSBNodelet*)context)->GetImage(buffer, buffersize);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(netusb_camera::NetUSBNodelet, nodelet::Nodelet);

