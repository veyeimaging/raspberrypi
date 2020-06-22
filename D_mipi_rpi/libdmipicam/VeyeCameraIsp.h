
#ifndef VEYECAMERAISP_H_
#define VEYECAMERAISP_H_


typedef struct
{
  //public
   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *isp_component;    /// Pointer to the isp component
   
   //private
    int height_align;//8 in default,for jpeg must align to 16
   //private
   char*    sensor_name; //reserved
   int      sensor_mode;//depending on sensors,Priority is higher than width and height
                        //0:1080p@30fps
                        //1: 720p@60fps
                        //2: VGA@120FPS
    int width;          // use to replace sensor_mode
    int height;         //  use to replace sensor_mode
} VEYE_CAMERA_ISP_STATE;


void veye_camera_isp_set_defaults(VEYE_CAMERA_ISP_STATE *state);

 MMAL_STATUS_T create_veye_camera_isp_component(VEYE_CAMERA_ISP_STATE *state,int cameraNum);

void destroy_veye_camera_isp_component(VEYE_CAMERA_ISP_STATE *state);


#endif


