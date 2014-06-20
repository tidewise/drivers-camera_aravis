#ifndef _CAMERA_ARAVIS_CAMERAARAVIS_HPP_
#define _CAMERA_ARAVIS_CAMERAARAVIS_HPP_

#include <iostream>
#include <string>
#include <camera_interface/CamInterface.h>
#include <arv.h>

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
			bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout=1000);
			bool isFrameAvailable();
			bool setCallbackFcn(void (*pcallback_function)(const void* p),void *p);
			bool isAttribAvail(const int_attrib::CamAttrib attrib);
			bool isAttribAvail(const double_attrib::CamAttrib attrib);
			bool isAttribAvail(const str_attrib::CamAttrib attrib);
			bool isAttribAvail(const enum_attrib::CamAttrib attrib);
			bool close();
			std::string doDiagnose();
			bool setAttrib(const int_attrib::CamAttrib attrib,const int value);
			int getAttrib(const int_attrib::CamAttrib attrib);
			bool setAttrib(const enum_attrib::CamAttrib attrib);
			bool isAttribSet(const enum_attrib::CamAttrib attrib);
			bool setFrameSettings(  const base::samples::frame::frame_size_t size, 
					const base::samples::frame::frame_mode_t mode,
					const uint8_t color_depth,
					const bool resize_frames);
		private:
			int camera_b_balance, camera_r_balance, camera_g_balance;
			std::string getBufferStatusString(ArvBufferStatus status);
			void printBufferStatus();
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
			int exposureFrameCounter;
			bool autoExposure;
			bool autoWhitebalance;
			int currentExposure;
			unsigned int payload;
			SimpleBrightnessIndicator brightnessIndicator;
			boost::shared_ptr<ExposureController> exposureController;
			pthread_mutex_t buffer_counter_lock;
			int buffer_counter;
			void (*callbackFcn)(const void* p);
			void *callbackData;
			base::samples::frame::frame_mode_t convertArvToFrameMode(ArvPixelFormat format);

			friend void aravisCameraCallback(ArvStream *stream, CameraAravis *driver);
	};

} 

#endif 
