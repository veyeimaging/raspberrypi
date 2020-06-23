
#ifndef VEYECAMERAISP_H_
#define VEYECAMERAISP_H_


typedef struct
{
  //public
   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *isp_component;    /// Pointer to the isp component
   
   //private
   int height_align;//8 in default,for jpeg must align to 16
   char*    sensor_name; //可以自行读取寄存器判断？预留
   
   int      sensor_mode;//depending on sensors
                        //0:1080p@30fps
                        ///1: 720p@60fps
                        ///2 vga 130fps
	int width;          // use to replace sensor_mode
	int height;         //  use to replace sensor_mode
	int framerate;       //frame rate
	MMAL_FOURCC_T out_yuv_fmt; //MMAL_ENCODING_I420 default
} VEYE_CAMERA_ISP_STATE;

void veye_camera_isp_set_defaults(VEYE_CAMERA_ISP_STATE *state);

 MMAL_STATUS_T create_veye_camera_isp_component(VEYE_CAMERA_ISP_STATE *state,int cameraNum);

void destroy_veye_camera_isp_component(VEYE_CAMERA_ISP_STATE *state);


#endif


