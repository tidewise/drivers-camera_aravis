#ifndef _CAMERA_ARAVIS_CAMERAARAVIS_HPP_
#define _CAMERA_ARAVIS_CAMERAARAVIS_HPP_

#include <iostream>
#include <string>
#include <camera_interface/CamInterface.h>
#include <arv.h>
#include <glog/logging.h>
#include <stdlib.h>

#include <camera_interface/BrightnessIndicator.h>
#include <camera_interface/ExposureController.h>
#include <boost/shared_ptr.hpp>


#include <semaphore.h>


namespace camera
{
	class CameraAravis : public CamInterface
	{
		public: 
			CameraAravis();
			~CameraAravis();
			bool grab(const GrabMode mode = SingleFrame, const int buffer_len=1);
			void openCamera(std::string camera_name);
			ArvPixelFormat getBayerFormat ();
			bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout=1000);
			bool isFrameAvailable();
			bool setCallbackFcn(void (*pcallback_function)(const void* p),void *p);
			bool setErrorCallbackFcn(void (*pcallback_function)(const void* p),void *p);
			bool isAttribAvail(const int_attrib::CamAttrib attrib);
			bool isAttribAvail(const double_attrib::CamAttrib attrib);
			bool isAttribAvail(const str_attrib::CamAttrib attrib);
			bool isAttribAvail(const enum_attrib::CamAttrib attrib);
			bool close();
			std::string doDiagnose();
			bool setAttrib(const int_attrib::CamAttrib attrib,const int value);
			int getAttrib(const int_attrib::CamAttrib attrib);
			bool setAttrib(const enum_attrib::CamAttrib attrib);
       	    		bool setAttrib(const double_attrib::CamAttrib attrib,const double value);
			bool isAttribSet(const enum_attrib::CamAttrib attrib);
			bool setFrameSettings(  const base::samples::frame::frame_size_t size, 
			const base::samples::frame::frame_mode_t mode,
			const uint8_t color_depth,
			const bool resize_frames);

		private:
			std::string getBufferStatusString(ArvBufferStatus status);
			void printBufferStatus();
			void startCapture();
			void stopCapture();
			void prepareBuffer(const size_t bufferLen);
			base::samples::frame::frame_mode_t convertArvToFrameMode(ArvPixelFormat format);

			friend void aravisCameraCallback(ArvStream *stream, CameraAravis *driver);
			friend void controlLostCallback (CameraAravis *driver);

                private:
                        std::string path;
			ArvCamera *camera;
			ArvStream *stream;
			int current_frame;
			int buffer_counter;
			void (*callbackFcn)(const void* p);
			void (*errorCallbackFcn)(const void* p);
			void *callbackData;
			void *errorCallbackData;
			pthread_mutex_t buffer_counter_lock;

			std::vector<base::samples::frame::Frame> camera_buffer;

                        unsigned long callback_handler;
                        unsigned long error_callback_handler;
	};

} 

#endif 
