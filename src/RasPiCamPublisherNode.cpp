#include <RasPiCamPublisherNode.hpp>

extern "C" {
#include "RaspiCLI.h"
} // C header

static XREF_T  exposureMode_map[] =
{
    {"auto", MMAL_PARAM_EXPOSUREMODE_AUTO},
    {"night", MMAL_PARAM_EXPOSUREMODE_NIGHT},
    {"nightpreview", MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW},
    {"backlight", MMAL_PARAM_EXPOSUREMODE_BACKLIGHT},
    {"spotlight", MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT},
    {"sports", MMAL_PARAM_EXPOSUREMODE_SPORTS},
    {"snow", MMAL_PARAM_EXPOSUREMODE_SNOW},
    {"beach", MMAL_PARAM_EXPOSUREMODE_BEACH},
    {"verylong", MMAL_PARAM_EXPOSUREMODE_VERYLONG},
    {"fixedfps", MMAL_PARAM_EXPOSUREMODE_FIXEDFPS},
    {"antishake", MMAL_PARAM_EXPOSUREMODE_ANTISHAKE},
    {"fireworks", MMAL_PARAM_EXPOSUREMODE_FIREWORKS},
};
static int exposureMode_map_size = sizeof(exposureMode_map) / sizeof(exposureMode_map[0]);

static XREF_T  awbMode_map[] =
{
    {"off",           MMAL_PARAM_AWBMODE_OFF},
    {"auto",          MMAL_PARAM_AWBMODE_AUTO},
    {"sunlight",      MMAL_PARAM_AWBMODE_SUNLIGHT},
    {"cloudy",        MMAL_PARAM_AWBMODE_CLOUDY},
    {"shade",         MMAL_PARAM_AWBMODE_SHADE},
    {"tungsten",      MMAL_PARAM_AWBMODE_TUNGSTEN},
    {"fluorescent",   MMAL_PARAM_AWBMODE_FLUORESCENT},
    {"incandescent",  MMAL_PARAM_AWBMODE_INCANDESCENT},
    {"flash",         MMAL_PARAM_AWBMODE_FLASH},
    {"horizon",       MMAL_PARAM_AWBMODE_HORIZON},
};
static int awbMode_map_size = sizeof(awbMode_map) / sizeof(awbMode_map[0]);

RasPiCamPublisher::RasPiCamPublisher(rclcpp::NodeOptions options)
  : Node("raspicam2", "camera", options.use_intra_process_comms(true)) 
  {
    state = std::make_shared<RASPIVID_STATE>();
    default_status(state.get());
    state->pNode = this;

    //declare all parameters
    declare_parameter("camera_id", 0);
    declare_parameter("image_transport");
    declare_parameter("show_preview", false);
    declare_parameter("width", state->common_settings.width);
    declare_parameter("height", state->common_settings.height);
    declare_parameter("fps");
    declare_parameter("quality");
    declare_parameter("enable_imv");
    declare_parameter("sharpness");
    declare_parameter("contrast");
    declare_parameter("brightness");
    declare_parameter("saturation");
    declare_parameter("ISO");
    declare_parameter("videoStabilisation");
    declare_parameter("exposureCompensation");
    declare_parameter("exposureMode");
    declare_parameter("flickerAvoidMode");
    declare_parameter("exposureMeterMode");
    declare_parameter("awbMode", std::string("auto"));
    declare_parameter("imageEffect");
    declare_parameter("colourEffects_enable");
    declare_parameter("colourEffects_u");
    declare_parameter("colourEffects_v");
    declare_parameter("rotation");
    declare_parameter("hflip");
    declare_parameter("vflip");
    declare_parameter("roi_x");
    declare_parameter("roi_y");
    declare_parameter("roi_w");
    declare_parameter("roi_h");
    declare_parameter("shutter_speed");
    declare_parameter("awb_gains_r");
    declare_parameter("awb_gains_b");
    declare_parameter("analog_gain");
    declare_parameter("digital_gain");

    // get parameters
    state->preview_parameters.wantPreview = get_parameter("show_preview").as_bool() ? 1 : 0;

    state->common_settings.width = get_parameter("width").as_int();
    state->common_settings.height = get_parameter("height").as_int();
    get_parameter_or("fps", state->framerate, state->framerate);
    get_parameter_or("quality", state->quality, state->quality);

    get_parameter_or("camera_id", state->common_settings.cameraNum, 0);
    RCLCPP_INFO(get_logger(), "Camera number %d.", state->common_settings.cameraNum);

    std::string image_transport;
    get_parameter_or<std::string>("image_transport", image_transport, "raw");
    state->raw_output = (image_transport=="raw");
    get_parameter_or("enable_imv", state->enable_imv_pub, false);

    buffer_callback_t cb_raw = nullptr;
    pub_img = nullptr;
    if(state->raw_output) {
        pub_img = create_publisher<sensor_msgs::msg::Image>("image", rclcpp::QoS(1));
        cb_raw = std::bind(&RasPiCamPublisher::onImageRaw, this, std::placeholders::_1, std::placeholders::_2);
        RCLCPP_INFO(get_logger(), "Raw enabled.");
    }

    buffer_callback_t cb_motion = nullptr;
    if(state->enable_imv_pub) {
        cb_motion = std::bind(&RasPiCamPublisher::onMotion, this, std::placeholders::_1, std::placeholders::_2);
        RCLCPP_INFO(get_logger(), "Motion vectors enabled.");
    }

    get_parameter_or("sharpness" , state->camera_parameters.sharpness , state->camera_parameters.sharpness );
    get_parameter_or("contrast"  , state->camera_parameters.contrast  , state->camera_parameters.contrast  );
    get_parameter_or("brightness", state->camera_parameters.brightness, state->camera_parameters.brightness);
    get_parameter_or("saturation", state->camera_parameters.saturation, state->camera_parameters.saturation);
    get_parameter_or("ISO", state->camera_parameters.ISO, 0);
    get_parameter_or("videoStabilisation", state->camera_parameters.videoStabilisation, 0);
    get_parameter_or("exposureCompensation", state->camera_parameters.exposureCompensation, 0);

    std::string paramValue;

    get_parameter_or<std::string>("exposureMode", paramValue, "auto");
    std::transform(paramValue.begin(), paramValue.end(), paramValue.begin(), [](unsigned char c){ return std::tolower(c); });
    state->camera_parameters.exposureMode = (MMAL_PARAM_EXPOSUREMODE_T)raspicli_map_xref(paramValue.c_str(), exposureMode_map, exposureMode_map_size);
    if( state->camera_parameters.exposureMode == -1)
    {
        RCLCPP_FATAL(get_logger(), "Invalid exposureMode " + paramValue);
        exit(1);
    }

    //get_parameter_or("flickerAvoidMode", state->camera_parameters.flickerAvoidMode, MMAL_PARAM_FLICKERAVOID_OFF);

    get_parameter_or<std::string>("exposureMeterMode", paramValue, "average");
    std::transform(paramValue.begin(), paramValue.end(), paramValue.begin(), [](unsigned char c){ return std::tolower(c); });
    if(!paramValue.compare("average"))
        state->camera_parameters.exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
    else if(!paramValue.compare("spot"))
        state->camera_parameters.exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;
    else if(!paramValue.compare("backlit"))
        state->camera_parameters.exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT;
    else if(!paramValue.compare("matrix"))
        state->camera_parameters.exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
    else
    {
        RCLCPP_FATAL(get_logger(), "Invalid exposureMeterMode " + paramValue);
        exit(1);
    }

    get_parameter_or<std::string>("awbMode", paramValue, "auto");
    std::transform(paramValue.begin(), paramValue.end(), paramValue.begin(), [](unsigned char c){ return std::tolower(c); });
    state->camera_parameters.awbMode = (MMAL_PARAM_AWBMODE_T)raspicli_map_xref(paramValue.c_str(), awbMode_map, awbMode_map_size);
    if( state->camera_parameters.awbMode == -1)
    {
        RCLCPP_FATAL(get_logger(), "Invalid awbMode " + paramValue);
        exit(1);
    }

    // get_parameter_or("imageEffect", state->camera_parameters.imageEffect, NONE);
    get_parameter_or("colourEffects_enable", state->camera_parameters.colourEffects.enable, state->camera_parameters.colourEffects.enable);
    get_parameter_or("colourEffects_u", state->camera_parameters.colourEffects.u, state->camera_parameters.colourEffects.u);
    get_parameter_or("colourEffects_v", state->camera_parameters.colourEffects.v, state->camera_parameters.colourEffects.v);
    get_parameter_or("rotation", state->camera_parameters.rotation, state->camera_parameters.rotation);
    get_parameter_or("hflip"   , state->camera_parameters.hflip   , state->camera_parameters.hflip   );
    get_parameter_or("vflip"   , state->camera_parameters.vflip   , state->camera_parameters.vflip   );
    get_parameter_or("roi_x"   , state->camera_parameters.roi.x   , state->camera_parameters.roi.x   );
    get_parameter_or("roi_y"   , state->camera_parameters.roi.y   , state->camera_parameters.roi.y   );
    get_parameter_or("roi_w"   , state->camera_parameters.roi.w   , state->camera_parameters.roi.w   );
    get_parameter_or("roi_h"   , state->camera_parameters.roi.h   , state->camera_parameters.roi.h   );
    get_parameter_or("shutter_speed", state->camera_parameters.shutter_speed, 0);

    get_parameter_or("awb_gains_r", state->camera_parameters.awb_gains_r, state->camera_parameters.awb_gains_r);
    get_parameter_or("awb_gains_b", state->camera_parameters.awb_gains_b, state->camera_parameters.awb_gains_b);

    // get_parameter_or("analog_gain", state->camera_parameters.awb_gains_r, 0.0f);
    // get_parameter_or("digital_gain", state->camera_parameters.awb_gains_b, 0.0f);

    pub_img_compressed = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", rclcpp::QoS(1));
    pub_info = create_publisher<sensor_msgs::msg::CameraInfo>("image/camera_info", rclcpp::QoS(1));
    srv_info = create_service<sensor_msgs::srv::SetCameraInfo>("set_camera_info",
        std::bind(&RasPiCamPublisher::set_camera_info, this,
        std::placeholders::_1, std::placeholders::_2));

    state->common_settings.verbose = 1;
    init_cam(*state,
             cb_raw,
             std::bind(&RasPiCamPublisher::onImageCompressed, this, std::placeholders::_1, std::placeholders::_2),
             cb_motion);


    double sensorWidth, sensorHeight, sensorFocalLength;
    const std::string cameraName(state->common_settings.camera_name);
    // set default camera parameters for Camera Module v2
    // https://www.raspberrypi.org/documentation/hardware/camera/
    // camera centre: 3.68 × 2.76 mm
    // focal length: 3.04 mm +/- 0.01
    if(!cameraName.compare("imx219"))
    {
        sensorWidth = 3.68;
        sensorHeight = 2.76;
        sensorFocalLength = 3.04;
    }
    // set default camera parameters for Camera Module v1
    // https://www.raspberrypi.org/documentation/hardware/camera/
    // camera centre: 3.76 × 2.74 mm
    // focal length: 3.60 mm +/- 0.01
    else 
    {
        sensorWidth = 3.76;
        sensorHeight = 2.74;
        sensorFocalLength = 3.60;
    }
    int w, h;
    w = state->common_settings.width;
    h = state->common_settings.height;
    const double cx = w/2.0;
    const double cy = h/2.0;
    const double fx = (sensorFocalLength / sensorWidth) * w;
    const double fy = (sensorFocalLength / sensorHeight) * h;
    camera_info.width = w;
    camera_info.height = h;
    camera_info.k = {fx, 0,  cx,
                     0,  fy, cy,
                     0,   0, 1};

    start_capture(*state);
}

RasPiCamPublisher::~RasPiCamPublisher() 
{
    close_cam(*state);
}

void RasPiCamPublisher::onImageRaw(const uint8_t* start, const uint8_t* end) 
{

    const auto tnow = now();

    sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
    msg->header.frame_id = "camera";
    msg->header.stamp = tnow;
    msg->height = camera_info.height;
    msg->width = camera_info.width;
    //calculating the step, must be for video capture a multiple of 16 (32 for stills)
    uint32_t rounded_width = (uint32_t)(ceil(msg->width/16.0) * 16);//aligned to 16
    msg->step = rounded_width * 3;
    msg->encoding = "rgb8";
    msg->data.insert(msg->data.end(), start, end);
    pub_img->publish(std::move(msg));

    camera_info.header.frame_id = "camera";
    camera_info.header.stamp = tnow;
    pub_info->publish(camera_info);
}

void RasPiCamPublisher::onImageCompressed(const uint8_t* start, const uint8_t* end) {

    const auto tnow = now();

    sensor_msgs::msg::CompressedImage::UniquePtr msg(new sensor_msgs::msg::CompressedImage());
    msg->header.frame_id = "camera";
    msg->header.stamp = tnow;
    // set raw compressed data
    msg->format = "jpeg";
    msg->data.insert(msg->data.end(), start, end);
    pub_img_compressed->publish(std::move(msg));

    camera_info.header.frame_id = "camera";
    camera_info.header.stamp = tnow;
    pub_info->publish(camera_info);
}

void RasPiCamPublisher::onMotion(const uint8_t* start, const uint8_t* end) { }

void RasPiCamPublisher::set_camera_info(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res)
{
    camera_info = req->camera_info;
    res->success = true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RasPiCamPublisher)
