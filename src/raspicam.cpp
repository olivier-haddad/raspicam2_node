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

// We use some GNU extensions (basename)
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <semaphore.h>

#include <stdbool.h>

extern "C" {
#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_parameters_camera.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/mmal_port.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/vcos/vcos.h"
#include "RaspiHelpers.h"
#include "RaspiPreview.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiCLI.h"
} // C header


#include <raspicam.h>
#include <rclcpp/rclcpp.hpp>

static constexpr int IMG_BUFFER_SIZE = 10 * 1024 * 1024;  // 10 MB

// Video format information
static constexpr int VIDEO_FRAME_RATE_DEN = 1;

// Video render needs at least 2 buffers.
static constexpr int VIDEO_OUTPUT_BUFFERS_NUM = 3;

int skip_frames = 0;

/**
 *  buffer header callback function for video encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) {
//   // We pass our own memory and other stuff in via the userdata field.
//   PORT_USERDATA* pData = port->userdata;

//   if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
//       // TODO: motion vectors
//     // Frame information
// //    motion_vectors.msg.header.seq = pData->frame;
// //    motion_vectors.msg.header.frame_id = camera_frame_id;
// //    motion_vectors.msg.header.stamp = ros::Time::now();

//     // Number of 16*16px macroblocks
// //    motion_vectors.msg.mbx = pData->pstate.width / 16;
//     if (pData->pstate.width % 16) {
// //      motion_vectors.msg.mbx++;
//     }

// //    motion_vectors.msg.mby = pData->pstate.height / 16;
//     if (pData->pstate.height % 16) {
// //      motion_vectors.msg.mby++;
//     }

//     mmal_buffer_header_mem_lock(buffer);

//     // Motion vector data
//     struct __attribute__((__packed__)) imv {
//       int8_t x;
//       int8_t y;
//       uint16_t sad;
//     }* imv = reinterpret_cast<struct imv*>(buffer->data);

//     size_t num_elements = buffer->length / sizeof(struct imv);
// //    motion_vectors.msg.x.resize(num_elements);
// //    motion_vectors.msg.y.resize(num_elements);
// //    motion_vectors.msg.sad.resize(num_elements);

//     for (size_t i = 0; i < num_elements; i++) {
// //      motion_vectors.msg.x[i] = imv->x;
// //      motion_vectors.msg.y[i] = imv->y;
// //      motion_vectors.msg.sad[i] = imv->sad;
//       imv++;
//     }

//     if(pData->callback!=nullptr) {
// //        const uint8_t* start = pData->buffer[pData->frame & 1].get();
// //        const uint8_t* end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
//         pData->callback(nullptr, nullptr);
//     }

//     mmal_buffer_header_mem_unlock(buffer);

// //    motion_vectors.pub.publish(motion_vectors.msg);
//     pData->frame++;
//   }

//   // release buffer back to the pool
//   mmal_buffer_header_release(buffer);

//   // and send one back to the port (if still open)
//   if (port->is_enabled) {
//     MMAL_STATUS_T status;

//     MMAL_BUFFER_HEADER_T* new_buffer = mmal_queue_get(pData->pstate.video_encoder_pool->queue);

//     if (new_buffer)
//       status = mmal_port_send_buffer(port, new_buffer);

//     if (!new_buffer || status != MMAL_SUCCESS) {
//       vcos_log_error("Unable to return a buffer to the video encoder port");
//       RCLCPP_ERROR(state.pNode->get_logger(), "Unable to return a buffer to the video encoder port");
//     }
//   }
}

/**
 * Create the image encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_image_encoder_component(RASPIVID_STATE& state) {
  MMAL_COMPONENT_T* encoder = 0;
  MMAL_PORT_T *encoder_input = nullptr, *encoder_output = nullptr;
  MMAL_STATUS_T status;
  MMAL_POOL_T* pool;

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to create image encoder component");
    RCLCPP_ERROR(state.pNode->get_logger(), "Unable to create image encoder component");
    goto error;
  }

  if (!encoder->input_num || !encoder->output_num) {
    status = MMAL_ENOSYS;
    vcos_log_error("Image encoder doesn't have input/output ports");
    RCLCPP_ERROR(state.pNode->get_logger(), "Image encoder doesn't have input/output ports");
    goto error;
  }

  encoder_input = encoder->input[0];
  encoder_output = encoder->output[0];

  // We want same format on input and output
  mmal_format_copy(encoder_output->format, encoder_input->format);

  // Only supporting H264 at the moment
  encoder_output->format->encoding = MMAL_ENCODING_JPEG;

  encoder_output->buffer_size = encoder_output->buffer_size_recommended;

  if (encoder_output->buffer_size < encoder_output->buffer_size_min)
    encoder_output->buffer_size = encoder_output->buffer_size_min;

  encoder_output->buffer_num = encoder_output->buffer_num_recommended;

  if (encoder_output->buffer_num < encoder_output->buffer_num_min)
    encoder_output->buffer_num = encoder_output->buffer_num_min;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(encoder_output);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on image encoder output port");
    RCLCPP_ERROR(state.pNode->get_logger(), "Unable to set format on image encoder output port");
    goto error;
  }

  // Set the JPEG quality level
  status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state.quality);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set JPEG quality");
    RCLCPP_ERROR(state.pNode->get_logger(), "Unable to set JPEG quality");
    goto error;
  }

  //  Enable component
  status = mmal_component_enable(encoder);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to enable image encoder component");
    RCLCPP_ERROR(state.pNode->get_logger(), "Unable to enable image encoder component");
    goto error;
  }

  /* Create pool of buffer headers for the output port to consume */
  pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

  if (!pool) {
    vcos_log_error("Failed to create buffer header pool for image encoder output port %s", encoder_output->name);
    RCLCPP_ERROR(state.pNode->get_logger(), "Failed to create buffer header pool for image encoder output port %s", encoder_output->name);
  }

  state.image_encoder_pool = mmal::pool_ptr(pool, [encoder](MMAL_POOL_T* ptr) {
    if (encoder->output[0] && encoder->output[0]->is_enabled) {
      mmal_port_disable(encoder->output[0]);
    }
    mmal_port_pool_destroy(encoder->output[0], ptr);
  });
  state.image_encoder_component.reset(encoder);

  RCLCPP_INFO(state.pNode->get_logger(), "Image encoder component done");

  return status;

error:
  if (encoder)
    mmal_component_destroy(encoder);

  return status;
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function
 * successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T* output_port, MMAL_PORT_T* input_port,
                                   mmal::connection_ptr& connection) {
  MMAL_STATUS_T status;

  MMAL_CONNECTION_T* new_connection = nullptr;

  status = mmal_connection_create(&new_connection, output_port, input_port,
                                  MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

  if (status == MMAL_SUCCESS) {
    status = mmal_connection_enable(new_connection);
    if (status != MMAL_SUCCESS)
      mmal_connection_destroy(new_connection);
  }

  connection.reset(new_connection);

  return status;
}


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
//#define VIDEO_FRAME_RATE_DEN 1

// Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms


/// Capture/Pause switch method
/// Simply capture for time specified
enum
{
   WAIT_METHOD_NONE,       /// Simply capture for time specified
   WAIT_METHOD_TIMED,      /// Cycle between capture and pause for times specified
   WAIT_METHOD_KEYPRESS,   /// Switch between capture and pause on keypress
   WAIT_METHOD_SIGNAL,     /// Switch between capture and pause on signal
   WAIT_METHOD_FOREVER     /// Run/record forever
};

/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T  profile_map[] =
{
   {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
   {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
   {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T  level_map[] =
{
   {"4",           MMAL_VIDEO_LEVEL_H264_4},
   {"4.1",         MMAL_VIDEO_LEVEL_H264_41},
   {"4.2",         MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);

static XREF_T  initial_map[] =
{
   {"record",     0},
   {"pause",      1},
};

static int initial_map_size = sizeof(initial_map) / sizeof(initial_map[0]);

static XREF_T  intra_refresh_map[] =
{
   {"cyclic",       MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
   {"adaptive",     MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
   {"both",         MMAL_VIDEO_INTRA_REFRESH_BOTH},
   {"cyclicrows",   MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
//   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

static XREF_T  raw_output_fmt_map[] =
{
   {"yuv",  RAW_OUTPUT_FMT_YUV},
   {"rgb",  RAW_OUTPUT_FMT_RGB},
   {"gray", RAW_OUTPUT_FMT_GRAY},
};

static int raw_output_fmt_map_size = sizeof(raw_output_fmt_map) / sizeof(raw_output_fmt_map[0]);

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
void default_status(RASPIVID_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));

   state->isInit=false;
   state->showPreview = false;

   // Now set anything non-zero
   state->common_settings.width = 1920;       // Default to 1080p
   state->common_settings.height = 1080;
   state->encoding = MMAL_ENCODING_H264;
   state->bitrate = 17000000; // This is a decent default bitrate for 1080p
   state->framerate = VIDEO_FRAME_RATE_NUM;
   state->intraperiod = -1;    // Not set
   state->quantisationParameter = 0;
   state->immutableInput = 1;
   state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   state->level = MMAL_VIDEO_LEVEL_H264_4;
   state->onTime = 5000;
   state->offTime = 5000;
   state->bCapturing = 0;
   state->bInlineHeaders = 0;
   state->inlineMotionVectors = 0;
   state->intra_refresh_type = -1;
   state->frame = 0;
   state->save_pts = 0;
   state->addSPSTiming = MMAL_FALSE;
   state->slices = 1;
   state->enable_raw_pub = false;
   state->raw_output_fmt = RAW_OUTPUT_FMT_RGB;

   raspicommonsettings_set_defaults(&state->common_settings);

   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

static void check_camera_model(int cam_num)
{
   MMAL_COMPONENT_T *camera_info;
   MMAL_STATUS_T status;

   // Try to get the camera name
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
   if (status == MMAL_SUCCESS)
   {
      MMAL_PARAMETER_CAMERA_INFO_T param;
      param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
      param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware version
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);

      if (status != MMAL_SUCCESS)
      {
         // Running on newer firmware
         param.hdr.size = sizeof(param);
         status = mmal_port_parameter_get(camera_info->control, &param.hdr);
         if (status == MMAL_SUCCESS && param.num_cameras > cam_num)
         {
            if (!strncmp(param.cameras[cam_num].camera_name, "toshh2c", 7))
            {
               fprintf(stderr, "The driver for the TC358743 HDMI to CSI2 chip you are using is NOT supported.\n");
               fprintf(stderr, "They were written for a demo purposes only, and are in the firmware on an as-is\n");
               fprintf(stderr, "basis and therefore requests for support or changes will not be acted on.\n\n");
            }
         }
      }

      mmal_component_destroy(camera_info);
   }
}

/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPIVID_STATE *state)
{
   int i;

   if (!state)
   {
      vcos_assert(0);
      return;
   }

   raspicommonsettings_dump_parameters(&state->common_settings);

   fprintf(stderr, "bitrate %d, framerate %d\n", state->bitrate, state->framerate);
   fprintf(stderr, "H264 Profile %s\n", raspicli_unmap_xref(state->profile, profile_map, profile_map_size));
   fprintf(stderr, "H264 Level %s\n", raspicli_unmap_xref(state->level, level_map, level_map_size));
   fprintf(stderr, "H264 Quantisation level %d, Inline headers %s\n", state->quantisationParameter, state->bInlineHeaders ? "Yes" : "No");
   fprintf(stderr, "H264 Fill SPS Timings %s\n", state->addSPSTiming ? "Yes" : "No");
   fprintf(stderr, "H264 Intra refresh type %s, period %d\n", raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size), state->intraperiod);
   fprintf(stderr, "H264 Slices %d\n", state->slices);

   if (state->enable_raw_pub)
      fprintf(stderr, "Raw output enabled, format %s\n", raspicli_unmap_xref(state->raw_output_fmt, raw_output_fmt_map, raw_output_fmt_map_size));

   fprintf(stderr, "\nInitial state '%s'\n", raspicli_unmap_xref(state->bCapturing, initial_map, initial_map_size));
   fprintf(stderr, "\n\n");

   raspipreview_dump_parameters(&state->preview_parameters);
   raspicamcontrol_dump_parameters(&state->camera_parameters);
}


/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void image_encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;

   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
   RASPIVID_STATE& state = *(pData->pstate);

   if (pData)
   {
      int bytes_written = buffer->length;

      if (buffer->length)
      {
         if (pData->id != INT_MAX) 
         {
            if (pData->id + buffer->length > IMG_BUFFER_SIZE) 
            {
               RCLCPP_ERROR(state.pNode->get_logger(), "pData->id (%d) + buffer->length (%d) > IMG_BUFFER_SIZE (%d), skipping the frame",                         pData->id, buffer->length, IMG_BUFFER_SIZE);
               pData->id = INT_MAX;  // mark this frame corrupted
            }
            else
            {
               mmal_buffer_header_mem_lock(buffer);
               memcpy(&(pData->buffer[pData->frame & 1].get()[pData->id]), buffer->data, buffer->length);
               pData->id += bytes_written;
               mmal_buffer_header_mem_unlock(buffer);
            }
         }
      }

      if (bytes_written != buffer->length) 
      {
         vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
         RCLCPP_ERROR(state.pNode->get_logger(), "Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
         pData->pstate->abort = true;
      }

      bool complete = false;
      if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
         complete = true;

      if (complete) 
      {
         if (pData->id != INT_MAX) 
         {
            // RCLCPP_INFO(state.pNode->get_logger(), "Frame size %d", pData->id);
            if (skip_frames > 0 && pData->frames_skipped < skip_frames) 
            {
               pData->frames_skipped++;
            }
            else
            {
               pData->frames_skipped = 0;
               if(pData->callback!=nullptr) 
               {
                  const uint8_t* start = pData->buffer[pData->frame & 1].get();
                  const uint8_t* end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
                  pData->callback(start, end);
               }
               pData->frame++;
            }
         }
         pData->id = 0;
      }
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->image_encoder_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}

/**
 *  buffer header callback function for splitter
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void splitter_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   PORT_USERDATA *pData = port->userdata;
   RASPIVID_STATE& state = *(pData->pstate);

   if (pData && pData->pstate->isInit)
   {
      int bytes_written = 0;
      int bytes_to_write = buffer->length;

      /* Write only luma component to get grayscale image: */
      if (buffer->length && pData->pstate->raw_output_fmt == RAW_OUTPUT_FMT_GRAY)
         bytes_to_write = port->format->es->video.width * port->format->es->video.height;

      if (bytes_to_write)
      {
         if (pData->id != INT_MAX) 
         {
            if (pData->id + buffer->length > IMG_BUFFER_SIZE)
            {
               RCLCPP_ERROR(state.pNode->get_logger(), "pData->id (%d) + buffer->length (%d) > IMG_BUFFER_SIZE (%d), skipping the frame", pData->id, buffer->length, IMG_BUFFER_SIZE);
               pData->id = INT_MAX;  // mark this frame corrupted
            }
            else
            {
               mmal_buffer_header_mem_lock(buffer);
               memcpy(&(pData->buffer[pData->frame & 1].get()[pData->id]), buffer->data, bytes_to_write);
               pData->id += bytes_to_write;
               bytes_written = bytes_to_write;
               mmal_buffer_header_mem_unlock(buffer);
            }
         }

         if (bytes_written != bytes_to_write)
         {
            vcos_log_error("Failed to write raw buffer data (%d from %d)- aborting", bytes_written, bytes_to_write);
            pData->pstate->abort = true;
         }

         int complete = false;
         if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
            complete = true;

         if (complete)
         {
            if (pData->id != INT_MAX)
            {
               // RCLCPP_INFO(state.pNode->get_logger(), "Frame size %d", pData->id);
               if (skip_frames > 0 && pData->frames_skipped < skip_frames)
               {
                  pData->frames_skipped++;
               }
               else
               {
                  pData->frames_skipped = 0;
                  if(pData->callback!=nullptr) 
                  {
                     const uint8_t* start = pData->buffer[pData->frame & 1].get();
                     const uint8_t* end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
                     pData->callback(start, end);
                  }
               }
            }
            pData->frame++;
            pData->id = 0;
         }
      }
   }
   else
   {
      vcos_log_error("Received a camera buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->splitter_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
      {
         RCLCPP_ERROR(state.pNode->get_logger(), "Unable to return a buffer to the splitter port");
      }
   }
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *video_port = nullptr, *preview_port = nullptr, *still_port = nullptr;
   MMAL_STATUS_T status;

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      RCLCPP_ERROR(state->pNode->get_logger(), "Failed to create camera component");
      goto error;
   }

   {
      int tempStatus = 0;
      tempStatus =  raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
      tempStatus += raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
      tempStatus += raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);
      status = (MMAL_STATUS_T)tempStatus;

      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Could not set stereo mode : error %d", status);
         goto error;
      }
   }

   {
      MMAL_PARAMETER_INT32_T camera_num =
      {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

      status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

      if (status != MMAL_SUCCESS)
      {
         RCLCPP_ERROR(state->pNode->get_logger(), "Could not select camera : error %d", status);
         goto error;
      }
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      RCLCPP_ERROR(state->pNode->get_logger(), "Camera doesn't have output ports");
      goto error;
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
      goto error;
   }

   preview_port = camera->output[mmal::camera_port::preview];
   video_port = camera->output[mmal::camera_port::video];
   still_port = camera->output[mmal::camera_port::capture];

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, default_camera_control_callback);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable control port : error %d", status);
      goto error;
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->common_settings.width,
         .max_stills_h = state->common_settings.height,
         .stills_yuv422 = 0,
         .one_shot_stills = 0,
         .max_preview_video_w = state->common_settings.width,
         .max_preview_video_h = state->common_settings.height,
         .num_preview_video_frames = 3 + vcos_max(0, (state->framerate-30)/10),
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 0,
         //.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC //MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
      };
      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   // Now set up the port formats

   // Set the encode format on the Preview port
   // HW limitations mean we need the preview to be the same size as the required recorded output

   format = preview_port->format;

   format->encoding = MMAL_ENCODING_I420;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 5, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 166, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }

   //enable dynamic framerate if necessary
   if (state->camera_parameters.shutter_speed)
   {
      if (state->framerate > 1000000./state->camera_parameters.shutter_speed)
      {
         state->framerate=0;
         if (state->common_settings.verbose)
            fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
      }
   }

   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(preview_port);

   if (status != MMAL_SUCCESS)
   {
      RCLCPP_ERROR(state->pNode->get_logger(), "camera preview format couldn't be set");
      goto error;
   }

   // Set the encode format on the video  port

   format = video_port->format;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 5, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 167, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }

   format->encoding = MMAL_ENCODING_I420;
//   format->encoding = MMAL_ENCODING_I420;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(video_port);

   if (status != MMAL_SUCCESS)
   {
      RCLCPP_ERROR(state->pNode->get_logger(), "camera video format couldn't be set");
      goto error;
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


   // Set the encode format on the still  port

   format = still_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);

   if (status != MMAL_SUCCESS)
   {
      RCLCPP_ERROR(state->pNode->get_logger(), "camera still format couldn't be set");
      goto error;
   }

   /* Ensure there are enough buffers to avoid dropping frames */
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS)
   {
      RCLCPP_ERROR(state->pNode->get_logger(), "camera component couldn't be enabled");
      goto error;
   }

   // Note: this sets lots of parameters that were not individually addressed before.
   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   state->camera_component = camera;

   if (state->common_settings.verbose)
      fprintf(stderr, "Camera component done\n");

   return status;

error:

   if (camera)
      mmal_component_destroy(camera);

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *splitter = 0;
   //for splitter raw output
   MMAL_PORT_T *splitter_output_imageenc_port = nullptr;
   MMAL_PORT_T *splitter_output_raw_port = nullptr;
   MMAL_ES_FORMAT_T *format;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;
   int i;

   if (state->camera_component == nullptr)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera component must be created before splitter");
      goto error;
   }

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create splitter component");
      RCLCPP_ERROR(state->pNode->get_logger(), "Unable to create image encoder component");
      goto error;
   }

   if (!splitter->input_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Splitter doesn't have any input port");
      RCLCPP_ERROR(state->pNode->get_logger(), "Video splitter doesn't have input ports");
      goto error;
   }

   if (splitter->output_num < 2)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Splitter doesn't have enough output ports");
      RCLCPP_ERROR(state->pNode->get_logger(), "Video splitter doesn't have enough output ports: %d", splitter->output_num);
      goto error;
   }

   /* Ensure there are enough buffers to avoid dropping frames: */
   mmal_format_copy(splitter->input[0]->format, state->camera_component->output[mmal::camera_port::video]->format);

   if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   status = mmal_port_format_commit(splitter->input[0]);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on splitter input port");
      RCLCPP_ERROR(state->pNode->get_logger(), "Unable to set format on splitter input port");
      goto error;
   }

   /*** Output to Encoder setup ***/
   splitter_output_imageenc_port = splitter->output[0];

   // Copy the format from the splitter input
   mmal_format_copy(splitter_output_imageenc_port->format, splitter->input[0]->format);

   status = mmal_port_format_commit(splitter_output_imageenc_port);

   if (status != MMAL_SUCCESS) {
      vcos_log_error("Unable to set format on splitter_output_imageenc_port for encoder");
      goto error;
   }
 
   /*** Output to raw setup. Splitter can do format conversion ***/
   splitter_output_raw_port = splitter->output[1];

   mmal_format_copy(splitter_output_raw_port->format, splitter->input[0]->format);

   format = splitter_output_raw_port->format;

   switch (state->raw_output_fmt)
   {
   case RAW_OUTPUT_FMT_YUV:
   case RAW_OUTPUT_FMT_GRAY: /* Grayscale image contains only luma (Y) component */
      format->encoding = MMAL_ENCODING_I420;
      format->encoding_variant = MMAL_ENCODING_I420;
      break;
   case RAW_OUTPUT_FMT_RGB:
      if (mmal_util_rgb_order_fixed(state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT]))
         format->encoding = MMAL_ENCODING_RGB24;
      else
         format->encoding = MMAL_ENCODING_BGR24;
      format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */
      break;
   default:
      status = MMAL_EINVAL;
      vcos_log_error("unknown raw output format");
      RCLCPP_ERROR(state->pNode->get_logger(), "unknown raw output format");
      goto error;
   }

   status = mmal_port_format_commit(splitter_output_raw_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on splitter_output_raw_port %d", i);
      RCLCPP_ERROR(state->pNode->get_logger(), "Unable to set format on splitter_output_raw_port %d", i);
      goto error;
   }

   /* copy the format for the rest of the ports */
   for (i = 2; i < splitter->output_num; i++)
   {
      mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);

      status = mmal_port_format_commit(splitter->output[i]);

      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set format on splitter output port %d", i);
         RCLCPP_ERROR(state->pNode->get_logger(), "Unable to set format on splitter output port %d", i);
         goto error;
      }
   }

   /* Enable component */
   status = mmal_component_enable(splitter);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("splitter component couldn't be enabled");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   //splitter_output = splitter->output[SPLITTER_OUTPUT_PORT];
   pool = mmal_port_pool_create(splitter_output_raw_port, splitter_output_raw_port->buffer_num, splitter_output_raw_port->buffer_size);
   RCLCPP_ERROR(state->pNode->get_logger(), "buffer size %d", splitter_output_raw_port->buffer_size);
   

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for splitter output port %s", splitter_output_raw_port->name);
      RCLCPP_ERROR(state->pNode->get_logger(), "Failed to create buffer header pool for image encoder output port %s", splitter_output_raw_port->name);
   }

   state->splitter_pool = pool;
   state->splitter_component = splitter;

   if (state->common_settings.verbose)
      fprintf(stderr, "Splitter component done\n");

   return status;

error:

   if (splitter)
      mmal_component_destroy(splitter);

   return status;
}

/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_splitter_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->splitter_pool)
   {
      mmal_port_pool_destroy(state->splitter_component->output[SPLITTER_OUTPUT_PORT], state->splitter_pool);
   }

   if (state->splitter_component)
   {
      mmal_component_destroy(state->splitter_component);
      state->splitter_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
//equivalent to create_video_encoder_component
static MMAL_STATUS_T create_video_encoder_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = nullptr, *encoder_output = nullptr;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
    RCLCPP_ERROR(state->pNode->get_logger(), "Unable to create video encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      RCLCPP_ERROR(state->pNode->get_logger(), "Video encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Only supporting H264 at the moment
   encoder_output->format->encoding = state->encoding;

   if(state->encoding == MMAL_ENCODING_H264)
   {
      if(state->level == MMAL_VIDEO_LEVEL_H264_4)
      {
         if(state->bitrate > MAX_BITRATE_LEVEL4)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            state->bitrate = MAX_BITRATE_LEVEL4;
         }
      }
      else
      {
         if(state->bitrate > MAX_BITRATE_LEVEL42)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
            state->bitrate = MAX_BITRATE_LEVEL42;
         }
      }
   }
   else if(state->encoding == MMAL_ENCODING_MJPEG)
   {
      if(state->bitrate > MAX_BITRATE_MJPEG)
      {
         fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
         state->bitrate = MAX_BITRATE_MJPEG;
      }
   }

   encoder_output->format->bitrate = state->bitrate;

   if (state->encoding == MMAL_ENCODING_H264)
      encoder_output->buffer_size = encoder_output->buffer_size_recommended;
   else
      encoder_output->buffer_size = 256<<10;


   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // We need to set the frame rate on output to 0, to ensure it gets
   // updated correctly from the input framerate when port connected
   encoder_output->format->es->video.frame_rate.num = 0;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      RCLCPP_ERROR(state->pNode->get_logger(), "Unable to set format on video encoder output port");
      goto error;
   }

   // Set the rate control parameter
   if (0)
   {
      MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set ratecontrol");
         goto error;
      }

   }

   if (state->encoding == MMAL_ENCODING_H264 && state->intraperiod != -1)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set intraperiod");
         goto error;
      }
   }

   if (state->encoding == MMAL_ENCODING_H264 && state->slices > 1 && state->common_settings.width <= 1280)
   {
      int frame_mb_rows = VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4;

      if (state->slices > frame_mb_rows) //warn user if too many slices selected
      {
         fprintf(stderr,"H264 Slice count (%d) exceeds number of macroblock rows (%d). Setting slices to %d.\n", state->slices, frame_mb_rows, frame_mb_rows);
         // Continue rather than abort..
      }
      int slice_row_mb = frame_mb_rows/state->slices;
      if (frame_mb_rows - state->slices*slice_row_mb)
         slice_row_mb++; //must round up to avoid extra slice if not evenly divided

      status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set number of slices");
         goto error;
      }
   }

   if (state->encoding == MMAL_ENCODING_H264 &&
       state->quantisationParameter)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set initial QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param2.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set min QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param3.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set max QP");
         goto error;
      }
   }

   if (state->encoding == MMAL_ENCODING_H264)
   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = (MMAL_VIDEO_PROFILE_T)state->profile;

      if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate > 245760)
      {
         if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate <= 522240)
         {
            fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
            state->level=MMAL_VIDEO_LEVEL_H264_42;
         }
         else
         {
            vcos_log_error("Too many macroblocks/s requested");
            status = MMAL_EINVAL;
            goto error;
         }
      }

      param.profile[0].level = (MMAL_VIDEO_LEVEL_T)state->level;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
         goto error;
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   if (state->encoding == MMAL_ENCODING_H264)
   {
      //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE HEADER FLAG parameters");
         // Continue rather than abort..
      }

      //set flag for add SPS TIMING
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
         // Continue rather than abort..
      }

      //set INLINE VECTORS flag to request motion vector estimates
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE VECTORS parameters");
         // Continue rather than abort..
      }

      // Adaptive intra refresh settings
      if ( state->intra_refresh_type != -1)
      {
         MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
         param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
         param.hdr.size = sizeof(param);

         // Get first so we don't overwrite anything unexpectedly
         status = mmal_port_parameter_get(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
            // Set some defaults, don't just pass random stack data
            param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
         }

         param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T)state->intra_refresh_type;

         //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
         //   param.cir_mbs = 10;

         status = mmal_port_parameter_set(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Unable to set H264 intra-refresh values");
            goto error;
         }
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      RCLCPP_ERROR(state->pNode->get_logger(), "Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
      RCLCPP_ERROR(state->pNode->get_logger(), "Failed to create buffer header pool for video encoder output port %s", encoder_output->name);
   }

   state->video_encoder_pool = mmal::pool_ptr(pool, [encoder](MMAL_POOL_T* ptr) {
      if (encoder->output[0] && encoder->output[0]->is_enabled) {
      mmal_port_disable(encoder->output[0]);
      }
      mmal_port_pool_destroy(encoder->output[0], ptr);
   });
   state->video_encoder_component.reset(encoder);
   RCLCPP_DEBUG(state->pNode->get_logger(), "Video encoder component done");

   return status;

error:
   if (encoder)
      mmal_component_destroy(encoder);

   return status;
}

int close_cam(RASPIVID_STATE& state) 
{
   if (state.common_settings.verbose)
      fprintf(stderr, "Closing down\n");

   state.isInit = false;

   MMAL_PORT_T *camera_still_port = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];;
   MMAL_PORT_T *encoder_output_port = state.image_encoder_component->output[0];
//   MMAL_PORT_T *splitter_output_port = state.splitter_component->output[SPLITTER_OUTPUT_PORT];
   MMAL_PORT_T *splitter_output_port = state.splitter_component->output[1];

   // Disable all our ports that are not handled by connections
   check_disable_port(camera_still_port);
   check_disable_port(encoder_output_port);
   check_disable_port(splitter_output_port);

   if (state.preview_parameters.wantPreview && state.preview_connection)
      mmal_connection_destroy(state.preview_connection);

   if (state.encoder_connection)
      mmal_connection_destroy(state.encoder_connection);

   if (state.splitter_connection)
      mmal_connection_destroy(state.splitter_connection);

   /* Disable components */
   if (state.image_encoder_component.get())
   {
      state.image_encoder_pool.reset();
      // Delete callback structure
      delete state.image_encoder_component->output[0]->userdata;
      state.image_encoder_component.reset();
   }
   state.image_encoder_connection.reset();

   if (state.video_encoder_component.get())
   {
      state.video_encoder_pool.reset();
      // Delete callback structure
      delete state.video_encoder_component->output[0]->userdata;
      state.video_encoder_component.reset();
   }

   if (state.preview_parameters.preview_component)
      mmal_component_disable(state.preview_parameters.preview_component);

   if (state.splitter_component)
   {
      mmal_component_disable(state.splitter_component);
      if(state.splitter_component->output[0]->userdata)
         delete state.splitter_component->output[0]->userdata;
      if(state.splitter_component->output[1]->userdata)
         delete state.splitter_component->output[1]->userdata;
   }

   if (state.camera_component)
      mmal_component_disable(state.camera_component);

   //destroy_encoder_component(&state);
   raspipreview_destroy(&state.preview_parameters);
   destroy_splitter_component(&state);
   destroy_camera_component(&state);

   if (state.common_settings.verbose)
      fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
   RCLCPP_INFO(state.pNode->get_logger(), "Video capture stopped");
}

int start_capture(RASPIVID_STATE& state)
{
  if (!(state.isInit))
    RCLCPP_FATAL(state.pNode->get_logger(), "Tried to start capture before camera is initialized");
  
  MMAL_PORT_T* camera_video_port = state.camera_component->output[mmal::camera_port::video];
  //video_encoder_output_port
  MMAL_PORT_T* image_encoder_output_port = state.image_encoder_component->output[0];
//  MMAL_PORT_T* splitter_output_raw = state.splitter_component->output[SPLITTER_OUTPUT_PORT];
  MMAL_PORT_T* splitter_output_raw_port = state.splitter_component->output[1];
  RCLCPP_INFO(state.pNode->get_logger(), "Starting video capture (%d, %d, %d, %d)", state.common_settings.width, state.common_settings.height, state.quality, state.framerate);

  // Send all the buffers to the image encoder output port
  {
   //  int num = mmal_queue_length(state.image_encoder_pool->queue);
   //  int q;
   //  for (q = 0; q < num; q++)
   //  {
   //    MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(state.image_encoder_pool->queue);

   //    if (!buffer) {
   //      vcos_log_error("Unable to get a required buffer %d from pool queue", q);
   //      RCLCPP_ERROR(state.pNode->get_logger(), "Unable to get a required buffer %d from pool queue", q);
   //    }

   //    if (mmal_port_send_buffer(image_encoder_output_port, buffer) != MMAL_SUCCESS) {
   //      vcos_log_error("Unable to send a buffer to image encoder output port (%d)", q);
   //      RCLCPP_ERROR(state.pNode->get_logger(), "Unable to send a buffer to image encoder output port (%d)", q);
   //    }
   //  }
  }
  // Send all the buffers to the video encoder output port
  //if (state.enable_imv_pub)
  if(state.enable_compressed_pub)
  {
    int num = mmal_queue_length(state.image_encoder_pool->queue);
    int q;
    for (q = 0; q < num; q++) 
    {
      MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(state.image_encoder_pool->queue);

      if (!buffer)
      {
        vcos_log_error("Unable to get a required buffer %d from pool queue", q);
        RCLCPP_ERROR(state.pNode->get_logger(), "Unable to get a required buffer %d from pool queue", q);
      }

      if (mmal_port_send_buffer(image_encoder_output_port, buffer) != MMAL_SUCCESS) 
      {
        vcos_log_error("Unable to send a buffer to video encoder output port (%d)", q);
        RCLCPP_ERROR(state.pNode->get_logger(), "Unable to send a buffer to video encoder output port (%d)", q);
      }
    }
  }

  // Send all the buffers to the splitter output port
  if (state.enable_raw_pub)
  {
    int num = mmal_queue_length(state.splitter_pool->queue);
    int q;
    for (q = 0; q < num; q++)
    {
      MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(state.splitter_pool->queue);

      if (!buffer)
      {
        vcos_log_error("Unable to get a required buffer %d from pool queue", q);
        RCLCPP_ERROR(state.pNode->get_logger(), "Unable to get a required buffer %d from pool queue", q);
      }

      if (mmal_port_send_buffer(splitter_output_raw_port, buffer) != MMAL_SUCCESS) 
      {
        vcos_log_error("Unable to send a buffer to splitter output port (%d)", q);
        RCLCPP_ERROR(state.pNode->get_logger(), "Unable to send a buffer to splitter output port (%d)", q);
      }
    }
  }

  if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
  {
    RCLCPP_FATAL(state.pNode->get_logger(), "Could not start video port capture.");
    return 1;
  }

  RCLCPP_INFO(state.pNode->get_logger(), "Video capture started");
  return 0;
}

int init_cam(RASPIVID_STATE& state, buffer_callback_t cb_raw, buffer_callback_t cb_compressed, buffer_callback_t cb_motion)
{
   int exit_code = EX_OK;

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T* camera_preview_port = nullptr;
   MMAL_PORT_T* camera_video_port = nullptr;
   MMAL_PORT_T *camera_still_port = nullptr;
   MMAL_PORT_T* preview_input_port = nullptr;
   MMAL_PORT_T* splitter_input_port = nullptr;

//   MMAL_PORT_T* splitter_output_enc = nullptr;
//   MMAL_PORT_T* splitter_output_raw = nullptr;
//   MMAL_PORT_T* image_encoder_input_port = nullptr;
//   MMAL_PORT_T* image_encoder_output_port = nullptr;
   //for motion vectors
//   MMAL_PORT_T* video_encoder_input_port = nullptr;
//   MMAL_PORT_T* video_encoder_output_port = nullptr;

   //for compressed images
   MMAL_PORT_T *encoder_input_port = nullptr;
   //for compressed images
   MMAL_PORT_T *encoder_output_port = nullptr;
   //for raw images
//   MMAL_PORT_T *splitter_output_port = nullptr;
   MMAL_PORT_T *splitter_preview_port = nullptr;

   MMAL_PORT_T *splitter_out_raw_port1 = nullptr;
   MMAL_PORT_T *splitter_out_compressed_port0 = nullptr;

   bcm_host_init();

   // Register our application with the logging system
   vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

   set_app_name("Raspicam_node");

   //already called in RasPiCamPublisher
   //default_status(&state);q

   // Setup for sensor specific parameters, only set W/H settings if zero on entry
   get_sensor_defaults(
      state.common_settings.cameraNum,
      state.common_settings.camera_name,
      &state.common_settings.width,
      &state.common_settings.height);

   if (state.common_settings.verbose)
      dump_status(&state);

   check_camera_model(state.common_settings.cameraNum);

   // OK, we have a nice set of parameters. Now set up our components
   // We have three components. Camera, Preview and encoder.

   if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create camera component", __func__);
      RCLCPP_ERROR(state.pNode->get_logger(), "%s: Failed to create camera component", __func__);
      exit_code = EX_SOFTWARE;
   }
   else if((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create preview component", __func__);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }
//   else if(state.enable_compressed_pub && (status = create_encoder_component(&state)) != MMAL_SUCCESS)
   // else if((status = create_encoder_component(&state)) != MMAL_SUCCESS)
   // {
   //    vcos_log_error("%s: Failed to create encode component", __func__);
   //    raspipreview_destroy(&state.preview_parameters);
   //    destroy_camera_component(&state);
   //    exit_code = EX_SOFTWARE;
   // }
   else if(state.enable_compressed_pub && (status = create_image_encoder_component(state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create image encoder component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }
   else if ((status = create_splitter_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create splitter component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      state.image_encoder_component.reset();
//      destroy_encoder_component(&state);
      exit_code = EX_SOFTWARE;
   }
   else
   {
      if (state.common_settings.verbose)
         fprintf(stderr, "Starting component connection stage\n");

      camera_preview_port = state.camera_component->output[mmal::camera_port::preview];
      camera_video_port   = state.camera_component->output[mmal::camera_port::video];
      camera_still_port   = state.camera_component->output[mmal::camera_port::capture];
      preview_input_port  = state.preview_parameters.preview_component->input[0];
      encoder_input_port  = state.image_encoder_component->input[0];
      encoder_output_port = state.image_encoder_component->output[0];

      // if (state.enable_raw_pub)
      // {
         splitter_input_port = state.splitter_component->input[0];
      //    splitter_output_port = state.splitter_component->output[SPLITTER_OUTPUT_PORT];
          //splitter_preview_port = state.splitter_component->output[SPLITTER_PREVIEW_PORT];
      // }

      splitter_out_raw_port1 = state.splitter_component->output[1];
      splitter_out_compressed_port0 = state.splitter_component->output[0];

      if (state.preview_parameters.wantPreview )
      {
//         if (state.enable_raw_pub)
         if(true)
         {
            if (state.common_settings.verbose)
//               fprintf(stderr, "Connecting camera preview port to splitter input port\n");
               fprintf(stderr, "Connecting camera video port to splitter input port\n");

            // Connect camera to splitter
//            status = connect_ports(camera_preview_port, splitter_input_port, &state.splitter_connection);
            status = connect_ports(camera_video_port, splitter_input_port, &state.splitter_connection);

            if (status != MMAL_SUCCESS)
            {
               state.splitter_connection = NULL;
//               vcos_log_error("%s: Failed to connect camera preview port to splitter input", __func__);
               vcos_log_error("%s: Failed to connect camera video port to splitter input", __func__);
               goto error;
            }

            if (state.common_settings.verbose)
            {
//               fprintf(stderr, "Connecting splitter preview port to preview input port\n");
               fprintf(stderr, "Connecting camera preview port to preview input port\n");
               fprintf(stderr, "Starting video preview\n");
            }

            // Connect splitter to preview
//            status = connect_ports(splitter_preview_port, preview_input_port, &state.preview_connection);
            status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);
         }
         else
         {
            if (state.common_settings.verbose)
            {
               fprintf(stderr, "Connecting camera preview port to preview input port\n");
               fprintf(stderr, "Starting video preview\n");
            }

            // Connect camera to preview
            status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);
         }

         if (status != MMAL_SUCCESS)
            state.preview_connection = NULL;
      }
      else
      {
//         if (state.enable_raw_pub)
         {
            if (state.common_settings.verbose)
               fprintf(stderr, "Connecting camera preview port to splitter input port\n");

            // Connect camera to splitter
            status = connect_ports(camera_preview_port, splitter_input_port, &state.splitter_connection);

            if (status != MMAL_SUCCESS)
            {
               state.splitter_connection = NULL;
               vcos_log_error("%s: Failed to connect camera preview port to splitter input", __func__);
               goto error;
            }
         }
         // else
         // {
         //    status = MMAL_SUCCESS;
         // }
      }

      if (status == MMAL_SUCCESS)
      {
         if(state.enable_compressed_pub)
         {
            if (state.common_settings.verbose)
               fprintf(stderr, "Connecting splitter to encoder input port\n");

            // Now connect the camera to the encoder
//            status = connect_ports(camera_video_port, encoder_input_port, &state.encoder_connection);
            status = connect_ports(splitter_out_compressed_port0, encoder_input_port, &state.encoder_connection);

            if (status != MMAL_SUCCESS)
            {
               state.encoder_connection = NULL;
               RCLCPP_ERROR(state.pNode->get_logger(), "%s: Failed to connect splitter port to encoder input", __func__);
               goto error;
            }
         }
      }

      if (status == MMAL_SUCCESS)
      {
         // Set up our userdata - this is passed though to the callback where we need the information.
         RCLCPP_INFO(state.pNode->get_logger(), "%s: setting up callback buffers", __func__);

         if (state.enable_raw_pub)
         {
            PORT_USERDATA* callback_data_raw = new PORT_USERDATA(&state);
            callback_data_raw->buffer[0] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
            callback_data_raw->buffer[1] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
            // Set up our userdata - this is passed though to the callback where we
            // need the information.
            callback_data_raw->id = 0;
            callback_data_raw->frame = 0;
            callback_data_raw->callback = cb_raw;

            splitter_out_raw_port1->userdata = callback_data_raw;

            if (state.common_settings.verbose)
               fprintf(stderr, "Enabling splitter output port\n");

            // Enable the splitter output port and tell it its callback function
//            status = mmal_port_enable(splitter_output_port, splitter_buffer_callback);
            status = mmal_port_enable(splitter_out_raw_port1, splitter_buffer_callback);

            if (status != MMAL_SUCCESS)
            {
               vcos_log_error("%s: Failed to setup splitter output port", __func__);
               goto error;
            }
         }

         if(state.enable_compressed_pub)
         {
            PORT_USERDATA* callback_data_enc = new PORT_USERDATA(&state);
            callback_data_enc->buffer[0] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
            callback_data_enc->buffer[1] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
            // Set up our userdata - this is passed though to the callback where we
            // need the information.
            callback_data_enc->id = 0;
            callback_data_enc->frame = 0;
            callback_data_enc->callback = cb_compressed;

            // Set up our userdata - this is passed though to the callback where we need the information.
            encoder_output_port->userdata = callback_data_enc;

            if (state.common_settings.verbose)
               fprintf(stderr, "Enabling encoder output port\n");

            // Enable the encoder output port and tell it its callback function
            status = mmal_port_enable(encoder_output_port, image_encoder_buffer_callback);

            if (status != MMAL_SUCCESS)
            {
               vcos_log_error("Failed to setup encoder output");
               goto error;
            }
         }

         state.isInit = true;
         RCLCPP_INFO(state.pNode->get_logger(), "%s: completed successfully.", __func__);
         return 0;
      }
      else
      {
         mmal_status_to_int(status);
         vcos_log_error("%s: Failed to connect camera to preview", __func__);
      }

error:
      mmal_status_to_int(status);

      close_cam(state);
   }

   if (status != MMAL_SUCCESS)
      raspicamcontrol_check_configuration(128);

   return exit_code;
}
