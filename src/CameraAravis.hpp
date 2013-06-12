#ifndef _CAMERA_ARAVIS_CAMERAARAVIS_HPP_
#define _CAMERA_ARAVIS_CAMERAARAVIS_HPP_

#include <iostream>
#include <string>
#include <camera_interface/CamInterface.h>
#include <arv.h>

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
			bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout=1000);
			bool setCallbackFcn(void (*pcallback_function)(const void* p),void *p);
		private:
			void startCapture();
			void stopCapture();
			void prepareBuffer(const size_t bufferLen);
			ArvCamera *camera; 
			ArvStream *stream;
			ArvPixelFormat format;
			base::samples::frame::Frame* camera_buffer;
			int current_frame;
			int buffer_len;
			int width, height;
			unsigned int payload;
			sem_t buffer_lock;
			void (*callbackFcn)(const void* p);
			void *callbackData;
			base::samples::frame::frame_mode_t convertArvToFrameMode(ArvPixelFormat format);

			friend void aravisCameraCallback(ArvStream *stream, CameraAravis *driver);
	};

} 

#endif 
