/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

extern "C" {
#include "interface/mmal/mmal.h"
#include "RaspiCamControl.h"
#include "RaspiCommonSettings.h"
#include "RaspiPreview.h"
}

#include "mmal_cxx_helper.h"

typedef std::function<void(const uint8_t*, const uint8_t*)> buffer_callback_t;

//FROM raspivid
// Forward
struct RASPIVID_STATE;
namespace rclcpp
{
  class Node;
}

typedef struct MMAL_PORT_USERDATA_T {
  MMAL_PORT_USERDATA_T(RASPIVID_STATE* state) : pstate(state){};
  std::unique_ptr<uint8_t[]> buffer[2];  // Memory to write buffer data to.
  int frame;
  int id;

  int frames_skipped = 0;

  buffer_callback_t callback = nullptr;
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
} PORT_USERDATA;

/** Possible raw output formats
 */
typedef enum
{
   RAW_OUTPUT_FMT_YUV = 0,
   RAW_OUTPUT_FMT_RGB,
   RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;
//END from raspivid

/** Structure containing all state information for the current run
 */
//OH raspivid
struct RASPIVID_STATE {
  RASPIVID_STATE()
    :
    showPreview (false)
     //camera_component(nullptr),
     //preview_component(nullptr)
    //, splitter_component(nullptr)
    , image_encoder_component(nullptr)
    , video_encoder_component(nullptr)
    //, splitter_connection(nullptr)
    , image_encoder_connection(nullptr)
    , video_encoder_connection(nullptr)
    //, splitter_pool(nullptr, mmal::default_delete_pool)
    , image_encoder_pool(nullptr, mmal::default_delete_pool)
    , video_encoder_pool(nullptr, mmal::default_delete_pool)
    , preview_connection(nullptr)
    {}
  rclcpp::Node *pNode = nullptr;
  
//   RASPICOMMONSETTINGS_PARAMETERS common_settings;     /// Common settings
  bool isInit;
  bool showPreview;
  //int width;      /// Requested width of image
  //int height;     /// requested height of image
  //int framerate;  /// Requested frame rate (fps)
  int quality;
  //bool enable_raw_pub;  // Enable Raw publishing
  bool enable_imv_pub;  // Enable publishing of inline motion vectors

  //int camera_id = 0;

  // RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
  // RASPICAM_CAMERA_PARAMETERS camera_parameters;  /// Camera setup parameters


//  mmal::component_ptr camera_component;
  //mmal::component_ptr preview_component;
  //mmal::component_ptr splitter_component;
  mmal::component_ptr image_encoder_component;
  mmal::component_ptr video_encoder_component;

//  mmal::connection_ptr splitter_connection;       /// Pointer to camera => splitter
  mmal::connection_ptr image_encoder_connection;  /// Pointer to splitter => encoder
  mmal::connection_ptr video_encoder_connection;  /// Pointer to camera => encoder
//  mmal::connection_ptr preview_connection;  /// Pointer to camera.previewport => preview

//  mmal::pool_ptr splitter_pool;       // Pointer buffer pool used by splitter (raw) output
  mmal::pool_ptr image_encoder_pool;  // Pointer buffer pool used by encoder (jpg) output
  mmal::pool_ptr video_encoder_pool;  // Pointer buffer pool used by encoder (h264) output
// };
// struct RASPIVID_STATE_S
// {
   RASPICOMMONSETTINGS_PARAMETERS common_settings;     /// Common settings
   MMAL_FOURCC_T encoding;             /// Requested codec video encoding (MJPEG or H264)
   int bitrate;                        /// Requested bitrate
   int framerate;                      /// Requested frame rate (fps)
   int intraperiod;                    /// Intra-refresh period (key frame rate)
   int quantisationParameter;          /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
   int bInlineHeaders;                  /// Insert inline headers to stream (SPS, PPS)
   int immutableInput;                 /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
   /// the camera output or the encoder output (with compression artifacts)
   int profile;                        /// H264 profile to use for encoding
   int level;                          /// H264 level to use for encoding
   int onTime;                         /// In timed cycle mode, the amount of time the capture is on per cycle
   int offTime;                        /// In timed cycle mode, the amount of time the capture is off per cycle

   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *splitter_component;  /// Pointer to the splitter component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera or splitter to preview
   MMAL_CONNECTION_T *splitter_connection;/// Pointer to the connection from camera to splitter
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *splitter_pool; /// Pointer to the pool of buffers used by splitter output port 0
   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

   //PORT_USERDATA callback_data;        /// Used to move data to the encoder callback

   int bCapturing;                     /// State of capture/pause
   bool abort;
   int bCircularBuffer;                /// Whether we are writing to a circular buffer

   int inlineMotionVectors;             /// Encoder outputs inline Motion Vectors
   char *imv_filename;                  /// filename of inline Motion Vectors output
   bool raw_output;                      /// Output raw video from camera as well
   RAW_OUTPUT_FMT raw_output_fmt;       /// The raw video format
   char *raw_filename;                  /// Filename for raw video output
   int intra_refresh_type;              /// What intra refresh type to use. -1 to not set.
   int frame;
   char *pts_filename;
   int save_pts;
   int64_t starttime;
   int64_t lasttime;

   MMAL_BOOL_T addSPSTiming;
   int slices;
};
/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state state structure to assign defaults to
 */
void default_status(RASPIVID_STATE *state);

/**
 * init_cam

 */
int init_cam(RASPIVID_STATE& state, buffer_callback_t cb_raw = nullptr, buffer_callback_t cb_compressed = nullptr, buffer_callback_t cb_motion = nullptr);

int start_capture(RASPIVID_STATE& state);

int close_cam(RASPIVID_STATE& state);
