#ifndef  _GNU_SOURCE 
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <libgen.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <semaphore.h>

#define VERSION_STRING "v0.1"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"

#include "VeyeCameraIsp.h"
#include "D_mipicam.h"

#define MAX_CAMERA_NUM 2
// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
// splitter port is preview usage
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

//最小bufer大小
#define MINI_BUFFER_SIZE  (2*1024*1024)
#define MINI_BUFFER_NUM 2

// Forward
typedef struct DCAMERA_S DCAMERA;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   DCAMERA *pcamera;              /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   char *cb_buff;                       /// Circular buffer
   int   cb_len;                        /// Length of buffer
   int   cb_wptr;                       /// Current write pointer
   int   cb_wrap;                       /// Has buffer wrapped at least once?
   int   cb_data;                       /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60*1000)
   int   iframe_buff[IFRAME_BUFSIZE];          /// buffer of iframe pointers
   int   iframe_buff_wpos;
   int   iframe_buff_rpos;
   char  header_bytes[29];
   int  header_wptr;
  // FILE *imv_file_handle;               /// File handle to write inline motion vectors to.
  // FILE *raw_file_handle;               /// File handle to write raw data to.
   int  flush_buffers;
 //  FILE *pts_file_handle;               /// File timestamps
   void * userdata;
} ENCODER_PORT_USERDATA;

/** Struct used to pass information in camera video port userdata to callback
 */
typedef struct
{
   DCAMERA *pcamera;           /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   int frame;
} COMM_PORT_USERDATA;

//preview encoder caputre yuvdata 四种功能同时只支持一种
//CAMERA_INSTANCE struct
typedef struct DCAMERA_S{
    struct camera_interface m_caminterface;
	
	//通用参数
	int width;
	int height;
	int framerate;
	VEYE_CAMERA_ISP_STATE	veye_camera_isp_state;
	MMAL_PORT_T *camera_output_port;
	MMAL_CONNECTION_T *isp_connection; /// Pointer to the connection from isp to camera
	 int verbose;                        /// !0 if want detailed run information
	//PORT_USERDATA callback_data;        /// Userdata
	
	//encoder 相关
    int	m_encoderstart;
    VIDEO_ENCODER_STATE	m_encoderstate;
    MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
    MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encode
    MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port
	OUTPUT_CALLBACK encoder_callback;
	ENCODER_PORT_USERDATA  encoder_userdata;
	int segmentSize;                    /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
   int segmentWrap;                    /// Point at which to wrap segment counter
   int segmentNumber;                  /// Current segment counter
   int splitNow;                       /// Split at next possible i-frame if set to 1.
   int splitWait;                      /// Switch if user wants splited files
   int inencodecallback;
	//void* encoder_userdata;

	//yuv stream相关
	int	m_yuvstreamstart;
	MMAL_POOL_T *yuvstream_pool;            /// Pointer to the pool of buffers used by camera video port
	OUTPUT_CALLBACK yuvstream_callback;
	COMM_PORT_USERDATA yuvstream_userdata;
	int inyuvcallback;
	//void* yuvstream_userdata;
	
	//preview相关
	int m_previewstart;
	PREVIEW_PARAMS preview_params;
	RASPIPREVIEW_PARAMETERS preview_parameters;    /// Preview setup parameters
	MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from isp to preview
	
	//capture相关
	int m_capturestart;
	IMAGE_FORMAT capture_param;
	MMAL_COMPONENT_T *capture_component;   /// Pointer to the encoder component
       MMAL_CONNECTION_T *capture_connection; /// Pointer to the connection from camera to encode
       MMAL_POOL_T *capture_pool; /// Pointer to the pool of buffers used by encoder output port
	//OUTPUT_CALLBACK capture_callback;
	COMM_PORT_USERDATA capture_userdata;
	VCOS_SEMAPHORE_T capture_complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
	//int capture_timeout;
	BUFFER capture_buffer;
	int capture_finish;
	
}DCAMERA;

DCAMERA	g_camera[MAX_CAMERA_NUM];

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING );
//   status =  mmal_connection_create(connection, output_port, input_port, 0);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}
static void default_status(DCAMERA *camera)
{
	if(camera == NULL)
		return;
	memset(camera,0,sizeof(DCAMERA));
	raspipreview_set_defaults(&camera->preview_parameters);
	camera->width = 1920;
	camera->height = 1080;
	camera->framerate = 30;
    camera->segmentSize = 0;  // 0 = not segmenting the file.
   camera->segmentNumber = 1;
   camera->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
   camera->splitNow = 0;
   camera->splitWait = 0;
   
	return;
}



/**
 * Create the encoder component, set up its ports
 *
 * @param camera Pointer to camera control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(DCAMERA *pcamera)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Only supporting H264 at the moment
   encoder_output->format->encoding = pcamera->m_encoderstate.encoding;

   if(pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264)
   {
      if(pcamera->m_encoderstate.level == MMAL_VIDEO_LEVEL_H264_4)
      {
         if(pcamera->m_encoderstate.bitrate > MAX_BITRATE_LEVEL4)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            pcamera->m_encoderstate.bitrate = MAX_BITRATE_LEVEL4;
         }
      }
      else
      {
         if(pcamera->m_encoderstate.bitrate > MAX_BITRATE_LEVEL42)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
            pcamera->m_encoderstate.bitrate = MAX_BITRATE_LEVEL42;
         }
      }
   }
   else if(pcamera->m_encoderstate.encoding == MMAL_ENCODING_MJPEG)
   {
      if(pcamera->m_encoderstate.bitrate > MAX_BITRATE_MJPEG)
      {
         fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
         pcamera->m_encoderstate.bitrate = MAX_BITRATE_MJPEG;
      }
   }
   
   encoder_output->format->bitrate = pcamera->m_encoderstate.bitrate;

   if (pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264)
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

   if (pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264 &&
       pcamera->m_encoderstate.intraperiod != -1)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, pcamera->m_encoderstate.intraperiod};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set intraperiod");
         goto error;
      }
   }

   if (pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264 &&
       pcamera->m_encoderstate.quantisationParameter)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, pcamera->m_encoderstate.quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set initial QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, pcamera->m_encoderstate.quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param2.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set min QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, pcamera->m_encoderstate.quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param3.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set max QP");
         goto error;
      }

   }

   if (pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264)
   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = pcamera->m_encoderstate.profile;

      if((VCOS_ALIGN_UP(pcamera->width,16) >> 4) * (VCOS_ALIGN_UP(pcamera->height,16) >> 4) * pcamera->framerate > 245760)
      {
         if((VCOS_ALIGN_UP(pcamera->width,16) >> 4) * (VCOS_ALIGN_UP(pcamera->height,16) >> 4) * pcamera->framerate <= 522240)
         {
            fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
            pcamera->m_encoderstate.level=MMAL_VIDEO_LEVEL_H264_42;
         }
         else
         {
            vcos_log_error("Too many macroblocks/s requested");
            goto error;
         }
      }
      
      param.profile[0].level = pcamera->m_encoderstate.level;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
         goto error;
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, pcamera->m_encoderstate.immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, pcamera->m_encoderstate.bInlineHeaders) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE HEADER FLAG parameters");
      // Continue rather than abort..
   }

   //set INLINE VECTORS flag to request motion vector estimates
   if (pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264 &&
       mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, pcamera->m_encoderstate.inlineMotionVectors) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE VECTORS parameters");
      // Continue rather than abort..
   }

   // Adaptive intra refresh settings
   if (pcamera->m_encoderstate.encoding == MMAL_ENCODING_H264 &&
       pcamera->m_encoderstate.intra_refresh_type != -1)
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

      param.refresh_mode = pcamera->m_encoderstate.intra_refresh_type;

      //if (pcamera->m_encoderstate.intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
      //   param.cir_mbs = 10;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 intra-refresh values");
         goto error;
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   pcamera->encoder_pool = pool;
   pcamera->encoder_component = encoder;

   if (pcamera->verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;

   error:
   if (encoder)
      mmal_component_destroy(encoder);

   pcamera->encoder_component = NULL;

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(DCAMERA *pcamera)
{
   // Get rid of any port buffers first
   if (pcamera->encoder_pool)
   {
      mmal_port_pool_destroy(pcamera->encoder_component->output[0], pcamera->encoder_pool);
   }

   if (pcamera->encoder_component)
   {
      mmal_component_destroy(pcamera->encoder_component);
      pcamera->encoder_component = NULL;
   }
}
/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}


int D_init_camera(CAMERA_INSTANCE *camera_instance, struct camera_interface cam_interface)
{
	MMAL_STATUS_T status = MMAL_SUCCESS;
	DCAMERA * pcamera = NULL;
	if( cam_interface.camera_num >= MAX_CAMERA_NUM)
		return -1;
	//default camera num
	if(cam_interface.camera_num == -1)
		pcamera = &g_camera[0];
	else
		pcamera = &g_camera[cam_interface.camera_num];
	//参数初始化
	default_status(pcamera);
	pcamera->m_caminterface = cam_interface;
	
	bcm_host_init();
   // Register our application with the logging system
   vcos_log_register("Dcamera", VCOS_LOG_CATEGORY);
   if ((status = create_veye_camera_isp_component(&pcamera->veye_camera_isp_state,cam_interface.camera_num)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create camera component", __func__);
      return -1;
   }
    pcamera->camera_output_port = pcamera->veye_camera_isp_state.camera_component->output[0];
	  // Note we are lucky that the preview and null sink components use the same input port
	  // so we can simple do this without conditionals
	 status = connect_ports(pcamera->camera_output_port, pcamera->veye_camera_isp_state.isp_component->input[0], &pcamera->isp_connection);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create rawcam->isp connection");
		goto error;
	}
	status = mmal_connection_enable(pcamera->isp_connection);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable rawcam->isp connection");
		goto error;
	}
   *camera_instance  = pcamera;
   
	return 0;
	error:
	D_close_camera(pcamera);
	return -1;
}
/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	
   MMAL_BUFFER_HEADER_T *new_buffer;
   static int64_t base_time =  -1;
   static int64_t last_second = -1;
	BUFFER userbuffer;
   // All our segment times based on the receipt of the first encoder callback
   if (base_time == -1)
      base_time = vcos_getmicrosecs64()/1000;

   // We pass our file handle and other stuff in via the userdata field.

    ENCODER_PORT_USERDATA *pData = (ENCODER_PORT_USERDATA *)port->userdata;
    pData->pcamera->inencodecallback = 1;
   if (pData)
   {
      int bytes_written = buffer->length;
      int64_t current_time = vcos_getmicrosecs64()/1000;
      //if(pData->pstate->inlineMotionVectors) vcos_assert(pData->imv_file_handle);
      {
         // For segmented record mode, we need to see if we have exceeded our time/size,
         // but also since we have inline headers turned on we need to break when we get one to
         // ensure that the new stream has the header in it. If we break on an I-frame, the
         // SPS/PPS header is actually in the previous chunk.
         if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
             ((pData->pcamera->segmentSize && current_time > base_time + pData->pcamera->segmentSize) ||
              (pData->pcamera->splitWait && pData->pcamera->splitNow)))
         {
            FILE *new_handle;

            base_time = current_time;

            pData->pcamera->splitNow = 0;
            pData->pcamera->segmentNumber++;

            // Only wrap if we have a wrap point set
            if (pData->pcamera->segmentWrap && pData->pcamera->segmentNumber > pData->pcamera->segmentWrap)
               pData->pcamera->segmentNumber = 1;

            
         }
         if (buffer->length)
         {
            mmal_buffer_header_mem_lock(buffer);
           /* if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
            {
               if(pData->pcamera->inlineMotionVectors)
               {
                  bytes_written = fwrite(buffer->data, 1, buffer->length, pData->imv_file_handle);
                  if(pData->flush_buffers) fflush(pData->imv_file_handle);
               }
               else
               {
                  //We do not want to save inlineMotionVectors...
                  bytes_written = buffer->length;
               }
            }
            else*/
            {
            //  vcos_log_error("write data %d ",buffer->length);
              // bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
			   if(pData->pcamera->encoder_callback)
			   {
				       userbuffer.data = buffer->data;
					userbuffer.alloc_size = buffer->alloc_size; 
					userbuffer.length = buffer->length;     
					userbuffer.flags = buffer->flags;      
					userbuffer.pts = buffer->pts;   
					userbuffer.userdata = pData->userdata;
				   pData->pcamera->encoder_callback(&userbuffer);
			   }
            }

            mmal_buffer_header_mem_unlock(buffer);

         }
      }

      // See if the second count has changed and we need to update any annotation
      if (current_time/1000 != last_second)
      {
       //  update_annotation_data(pData->pstate);
         last_second = current_time/1000;
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

      new_buffer = mmal_queue_get(pData->pcamera->encoder_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
   pData->pcamera->inencodecallback = 0;
}

int D_start_video_stream(CAMERA_INSTANCE camera_instance, VIDEO_ENCODER_STATE *encoder_state, OUTPUT_CALLBACK callback, void *userdata)
{
	MMAL_STATUS_T status = MMAL_SUCCESS;
	MMAL_PORT_T *encoder_output_port = NULL;
	if(camera_instance == NULL || encoder_state == NULL)
		return -1;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	memcpy(&pcamera->m_encoderstate,encoder_state,sizeof(VIDEO_ENCODER_STATE));
	pcamera->encoder_callback = callback;
	//pcamera->encoder_userdata = userdata;
	 if ((status = create_encoder_component(pcamera)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create encode component", __func__);
      return -1;
   }
    encoder_output_port = pcamera->encoder_component->output[0];

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create rawcam->isp connection");
		goto error;
	} 	
	 // Now connect the camera to the encoder
	 status = connect_ports(pcamera->veye_camera_isp_state.isp_component->output[0], pcamera->encoder_component->input[0], &pcamera->encoder_connection);

	 if (status != MMAL_SUCCESS)
	 {
		pcamera->encoder_connection = NULL;
		vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
		goto error;
	 }
	  pcamera->encoder_userdata.pcamera = pcamera;
	  pcamera->encoder_userdata.abort = 0;
	  pcamera->encoder_userdata.userdata = userdata;
	  //pcamera->encoder_userdata.file_handle = NULL;

	 // Set up our userdata - this is passed though to the callback where we need the information.
	 encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&pcamera->encoder_userdata;

	 if (pcamera->verbose)
		fprintf(stderr, "Enabling encoder output port\n");

	 // Enable the encoder output port and tell it its callback function
	 status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

	 if (status != MMAL_SUCCESS)
	 {
		vcos_log_error("Failed to setup encoder output");
		goto error;
	 }
	
	  int num = mmal_queue_length(pcamera->encoder_pool->queue);
	  int q;
	  for (q=0;q<num;q++)
	  {
		 MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pcamera->encoder_pool->queue);

		 if (!buffer)
			vcos_log_error("Unable to get a required buffer %d from pool queue", q);

		 if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
			vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
	  }
	  //ok then
	pcamera->m_encoderstart = 1;
	return 0;
	
	error:
	D_stop_video_stream(camera_instance);
	return -1;
	
}

int D_stop_video_stream(CAMERA_INSTANCE camera_instance)
{
	MMAL_PORT_T *encoder_output_port = NULL;
	if(camera_instance == NULL)
		return -1;
	
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	
	encoder_output_port = pcamera->encoder_component->output[0];
	//ensure call back finish first
	while(pcamera->inencodecallback)
		usleep(10);
	
	check_disable_port(encoder_output_port);
    if (pcamera->encoder_connection)
      mmal_connection_destroy(pcamera->encoder_connection);
  /* Disable components */
      if (pcamera->encoder_component)
         mmal_component_disable(pcamera->encoder_component);
	destroy_encoder_component(pcamera);
	pcamera->m_encoderstart = 0;
	return 0;
}

/**
 *  buffer header callback function for camera
 *
 *  Callback will dump buffer data to internal buffer
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void yuv_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   BUFFER userbuffer;
   // We pass our file handle and other stuff in via the userdata field.
   COMM_PORT_USERDATA *pData = (COMM_PORT_USERDATA *)port->userdata;
   if (pData)
   {
      int bytes_to_write = buffer->length;
      vcos_log_error("get one frame len %d", bytes_to_write);

      if (bytes_to_write)
      {
         mmal_buffer_header_mem_lock(buffer);
         //bytes_written = fwrite(buffer->data, 1, bytes_to_write, pData->pcamera->file_handle);
		 if(pData->pcamera->yuvstream_callback)
		{
		    userbuffer.data = buffer->data;
			userbuffer.alloc_size = buffer->alloc_size; 
			userbuffer.length = buffer->length;     
			userbuffer.flags = buffer->flags;      
			userbuffer.pts = buffer->pts;   
		    pData->pcamera->yuvstream_callback(&userbuffer);
		}
		 
         mmal_buffer_header_mem_unlock(buffer);
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

      new_buffer = mmal_queue_get(pData->pcamera->yuvstream_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the camera port");
   }
}

int D_start_yuv_stream(CAMERA_INSTANCE camera_instance, OUTPUT_CALLBACK callback, void *userdata)
{
	MMAL_STATUS_T status = MMAL_SUCCESS;
	MMAL_PORT_T *camera_video_port = NULL;
	if(camera_instance == NULL)
		return -1;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	pcamera->yuvstream_callback = callback;
	camera_video_port = pcamera->veye_camera_isp_state.isp_component->output[0];
    /* Create pool of buffer headers for the output port to consume */
    pcamera->yuvstream_pool = mmal_port_pool_create(camera_video_port, camera_video_port->buffer_num, camera_video_port->buffer_size);
    if (!pcamera->yuvstream_pool)
    {
	  vcos_log_error("Failed to create buffer header pool for encoder output port %s", camera_video_port->name);
    }
	pcamera->yuvstream_userdata.pcamera = pcamera;
	pcamera->yuvstream_userdata.abort = 0;
	 // Set up our userdata - this is passed though to the callback where we need the information.
	 camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&pcamera->yuvstream_userdata;

  // Enable the camera video port and tell it its callback function
	 status = mmal_port_enable(camera_video_port, yuv_buffer_callback);

	 if (status != MMAL_SUCCESS)
	 {
		vcos_log_error("Failed to setup camera output");
		goto error;
	 }
	int num = mmal_queue_length(pcamera->yuvstream_pool->queue);
	int q;
	for (q=0;q<num;q++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pcamera->yuvstream_pool->queue);
		if (!buffer)
			vcos_log_error("Unable to get a required buffer %d from pool queue", q);
		if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
			vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
	}
	pcamera->m_yuvstreamstart = 1;
	return 0;
	
	error:
	D_stop_yuv_stream(camera_instance);
	return -1;
}

int D_stop_yuv_stream(CAMERA_INSTANCE camera_instance)
{
	if(camera_instance == NULL)
		return -1;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	MMAL_PORT_T *camera_video_port = NULL;
	
	camera_video_port = pcamera->veye_camera_isp_state.isp_component->output[0];
	//ensure call back finish first
	while(pcamera->inyuvcallback)
		usleep(10);
	
	check_disable_port(camera_video_port);
	if(pcamera->yuvstream_pool)
	{
	 mmal_port_pool_destroy(camera_video_port, pcamera->yuvstream_pool);
	}
	pcamera->m_yuvstreamstart = 0;
	return 0;
}
/**
 *  buffer header callback function for camera output port
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void capture_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   int complete = 0;
	//   BUFFER userbuffer;
    // We pass our file handle and other stuff in via the userdata field.
   COMM_PORT_USERDATA *pData = (COMM_PORT_USERDATA *)port->userdata;

   if (pData)
   {
      int bytes_to_write = buffer->length;

	 vcos_log_error("capture_buffer_callback data len is %d ",bytes_to_write);
	
      if (bytes_to_write)
      {
         mmal_buffer_header_mem_lock(buffer);
		 pData->pcamera->capture_buffer.priv = buffer;
		 pData->pcamera->capture_buffer.data = buffer->data;
		 pData->pcamera->capture_buffer.alloc_size = buffer->alloc_size; 
		 pData->pcamera->capture_buffer.length = buffer->length;     
		 pData->pcamera->capture_buffer.flags = buffer->flags;      
		 pData->pcamera->capture_buffer.pts = buffer->pts;   
         mmal_buffer_header_mem_unlock(buffer);
		 pData->pcamera->capture_finish = 1;
      }

   }
   {
      vcos_semaphore_post(&(pData->pcamera->capture_complete_semaphore));
   }
}


/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct. encoder_component member set to the created camera_component if successful.
 *
 * @return a MMAL_STATUS, MMAL_SUCCESS if all OK, something else otherwise
 */
static MMAL_STATUS_T create_image_encoder_component(DCAMERA *pcamera)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create JPEG encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("JPEG encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Specify out output format
   encoder_output->format->encoding = pcamera->capture_param.encoding;

   encoder_output->buffer_size = encoder_output->buffer_size_recommended;


   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

	if(MMAL_ENCODING_BMP == pcamera->capture_param.encoding || MMAL_ENCODING_PNG == pcamera->capture_param.encoding )
	{	
		encoder_output->buffer_size = pcamera->width*pcamera->height*3+54;
	}
	else
	{
		if (encoder_output->buffer_size < MINI_BUFFER_SIZE)
		      encoder_output->buffer_size =MINI_BUFFER_SIZE;
	}
	if (encoder_output->buffer_num < MINI_BUFFER_NUM)
	      encoder_output->buffer_num = MINI_BUFFER_NUM;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      goto error;
   }

   // Set the JPEG quality level
   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, pcamera->capture_param.quality);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set JPEG quality");
      goto error;
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status  != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }
   //vcos_log_error("create state encodeing %d buffer header pool for encoder %d output buffer num %d len %d",state->encoding, encoder_output->buffer_size_recommended,encoder_output->buffer_num,encoder_output->buffer_size);
   pcamera->capture_pool = pool;
   pcamera->capture_component = encoder;

   if (pcamera->verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;

   error:

   if (encoder)
      mmal_component_destroy(encoder);

   return status;
}

/*
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_image_encoder_component(DCAMERA *pcamera)
{
   //Get rid of any port buffers first
   if(pcamera->capture_pool)
   {
      mmal_port_pool_destroy(pcamera->capture_component->output[0], pcamera->capture_pool);
   }

   if (pcamera->capture_component)
   {
      mmal_component_destroy(pcamera->capture_component);
      pcamera->capture_component = NULL;
   }
}

static int sem_timedwait_millsecs(sem_t *sem, long msecs)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	long secs = msecs/1000;
	msecs = msecs%1000;
	
	long add = 0;
	msecs = msecs*1000*1000 + ts.tv_nsec;
	add = msecs / (1000*1000*1000);
	ts.tv_sec += (add + secs);
	ts.tv_nsec = msecs%(1000*1000*1000);
 
	return sem_timedwait(sem, &ts);
}

BUFFER *D_capture(CAMERA_INSTANCE camera_instance, IMAGE_FORMAT *format, int timeout)
{
	if(camera_instance == NULL || format == NULL)
		return NULL;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	
	MMAL_STATUS_T status = MMAL_SUCCESS;
	MMAL_PORT_T *encoder_output_port = NULL;

	memcpy(&pcamera->capture_param,format,sizeof(IMAGE_FORMAT));
	//pcamera->capture_callback = capture_buffer_callback;
	if(format->encoding != IMAGE_ENCODING_I420 && format->encoding != IMAGE_ENCODING_I422)
	{
		 if ((status = create_image_encoder_component(pcamera)) != MMAL_SUCCESS)
		{
		  vcos_log_error("%s: Failed to create encode component", __func__);
		  return NULL;
		}
		encoder_output_port = pcamera->capture_component->output[0];

		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to create rawcam->isp connection");
			goto error;
		} 	
		 // Now connect the camera to the encoder
		 status = connect_ports(pcamera->veye_camera_isp_state.isp_component->output[0], pcamera->capture_component->input[0], &pcamera->capture_connection);

		 if (status != MMAL_SUCCESS)
		 {
			pcamera->capture_connection = NULL;
			vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
			goto error;
		 }
	}
	else
	{
		pcamera->capture_component = pcamera->veye_camera_isp_state.isp_component;
		encoder_output_port = pcamera->capture_component->output[0];
		/* Create pool of buffer headers for the output port to consume */
		pcamera->capture_pool = mmal_port_pool_create(encoder_output_port, encoder_output_port->buffer_num, encoder_output_port->buffer_size);
		if (!pcamera->capture_pool)
		{
		  vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output_port->name);
		}
		
	}
	 VCOS_STATUS_T vcos_status;
	 vcos_status = vcos_semaphore_create(&pcamera->capture_complete_semaphore, "Dcapture-sem", 0);
        vcos_assert(vcos_status == VCOS_SUCCESS);
	 
	 
	  pcamera->capture_userdata.pcamera = pcamera;
	  pcamera->capture_userdata.abort = 0;
	  //pcamera->capture_userdata.file_handle = NULL;

	 // Set up our userdata - this is passed though to the callback where we need the information.
	 encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&pcamera->capture_userdata;

	 if (pcamera->verbose)
		fprintf(stderr, "Enabling encoder output port\n");

	 // Enable the encoder output port and tell it its callback function
	 status = mmal_port_enable(encoder_output_port, capture_buffer_callback);

	 if (status != MMAL_SUCCESS)
	 {
		vcos_log_error("Failed to setup encoder output");
		goto error;
	 }
	
	  int num = mmal_queue_length(pcamera->capture_pool->queue);
	  int q;
	  for (q=0;q<num;q++)
	  {
		 MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pcamera->capture_pool->queue);

		 if (!buffer)
			vcos_log_error("Unable to get a required buffer %d from pool queue", q);

		 if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
			vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
	  }
	  struct timespec time;
	
	  //wait for capture finish
	  sem_timedwait_millsecs(&pcamera->capture_complete_semaphore,timeout);
	
	if(pcamera->capture_finish)
	{
		return &pcamera->capture_buffer;
	}
	
	error:
	D_release_buffer(camera_instance,&pcamera->capture_buffer);
	return NULL;
	
}


void D_release_buffer(CAMERA_INSTANCE camera_instance,BUFFER *buffer)
{
	if(camera_instance==NULL || buffer == NULL)
		return ;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	// release buffer back to the pool
      mmal_buffer_header_release((MMAL_BUFFER_HEADER_T*)buffer->priv);
	
	//check_disable_port(camera_video_port);
 	/*int num = mmal_queue_length(pcamera->capture_pool->queue);
	  int q;
	  for (q=0;q<num;q++)
	  {
		 MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pcamera->capture_pool->queue);

		 if (!buffer)
			vcos_log_error("Unable to get a required buffer %d from pool queue", q);

		 if (mmal_port_send_buffer(pcamera->capture_component->output[0], buffer)!= MMAL_SUCCESS)
			vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
	  }*/
	
	check_disable_port(pcamera->capture_component->output[0]);
    
	if(pcamera->capture_param.encoding != IMAGE_ENCODING_I420 && pcamera->capture_param.encoding != IMAGE_ENCODING_I422)
	{
		if (pcamera->capture_connection)
	      mmal_connection_destroy(pcamera->capture_connection);
		/* Disable components */
		if (pcamera->capture_component)
			mmal_component_disable(pcamera->capture_component);
		destroy_image_encoder_component(pcamera);
	}
	else
	{
		if(pcamera->capture_pool)
		{
		 	mmal_port_pool_destroy(pcamera->capture_component->output[0], pcamera->capture_pool);
		}
	}
	pcamera->m_encoderstart = 0;
	return ;

}

int D_start_preview(CAMERA_INSTANCE camera_instance, PREVIEW_PARAMS *preview_params)
{
	MMAL_STATUS_T status = MMAL_SUCCESS;
	if(camera_instance == NULL)
		return -1;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	pcamera->preview_parameters.wantPreview = 1;
	pcamera->preview_parameters.wantFullScreenPreview = preview_params->fullscreen;
	pcamera->preview_parameters.opacity = preview_params->opacity;
	pcamera->preview_parameters.previewWindow.x = preview_params->window.x;
	pcamera->preview_parameters.previewWindow.y = preview_params->window.y;
	pcamera->preview_parameters.previewWindow.width = preview_params->window.width;
	pcamera->preview_parameters.previewWindow.height = preview_params->window.height;
	if ((status = raspipreview_create(&pcamera->preview_parameters)) != MMAL_SUCCESS)
	   {
	      vcos_log_error("%s: Failed to create preview component", __func__);
	     return -1;
	   }
	  // Connect camera to preview (which might be a null_sink if no preview required)
		 status = connect_ports( pcamera->veye_camera_isp_state.isp_component->output[0], pcamera->preview_parameters.preview_component->input[0], &pcamera->preview_connection);
	if (status != MMAL_SUCCESS)
	   {
		vcos_log_error("Failed to create isp->render connection");
		goto error;
	}
	
	status = mmal_connection_enable(pcamera->preview_connection);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable isp->render connection");
		goto error;
	}
   
	pcamera->m_previewstart = 1;
	return 0;
	
	error:
	D_stop_preview(camera_instance);
	return -1;
}


int D_stop_preview(CAMERA_INSTANCE camera_instance)
{
	if(camera_instance == NULL)
		return -1;
	
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	if(pcamera->m_previewstart ==0)
		return -1;
   if (pcamera->preview_connection)
	  mmal_connection_destroy(pcamera->preview_connection);
  
  /* Disable components */
  if (pcamera->preview_parameters.preview_component)
	 mmal_component_disable(pcamera->preview_parameters.preview_component);
    raspipreview_destroy(&pcamera->preview_parameters);
	
	pcamera->m_previewstart = 0;
	return 0;
}


int D_close_camera(CAMERA_INSTANCE camera_instance)
{
	if(camera_instance == NULL)
		return -1;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	 if (pcamera->isp_connection)
	  mmal_connection_destroy(pcamera->isp_connection);

	destroy_veye_camera_isp_component(&pcamera->veye_camera_isp_state);
	
	return 0;
}


int D_get_support_formats(CAMERA_INSTANCE camera_instance, struct format *fmt, int index)
{
	if(camera_instance == NULL || fmt == NULL)
		return -1;
	DCAMERA * pcamera = (DCAMERA *)camera_instance;
	//just support one type now!
	if(index == 0)
	{
		fmt->width = 1920;
		fmt->height = 1080;
		fmt->maxframrate = 30; //此分辨率下最大帧率
		return 0;
	}
	return -1;
	
}


