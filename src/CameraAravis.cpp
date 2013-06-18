#include "CameraAravis.hpp"

#include <exception>
#include <semaphore.h>

using namespace std;

namespace camera
{
	void aravisCameraCallback(ArvStream *stream, CameraAravis *driver) {
		cout << "aravisCameraCallback" << endl;
		//TODO: Irgendwie sind Semaphoren doch ekelig und unötig, man kann auch direkt die Library-Funktionen von Aravis nutzen...
		pthread_mutex_lock(&(driver->buffer_counter_lock));
		driver->buffer_counter++;
		pthread_mutex_unlock(&(driver->buffer_counter_lock));
		if(driver->callbackFcn != 0) {
			driver->callbackFcn(driver->callbackData);
		}
	}

	CameraAravis::CameraAravis() {
		g_type_init();

		camera = 0;
		stream = 0;
		camera_buffer = 0;
		current_frame = 0;
		buffer_len = 0;
		callbackFcn = 0;
		buffer_counter = 0;
		pthread_mutex_init(&buffer_counter_lock, NULL);
	}
	CameraAravis::~CameraAravis() {
		if(camera_buffer != 0) {
			delete[] camera_buffer;
		}
	}

	void CameraAravis::openCamera(std::string camera_name) {
		camera = arv_camera_new(camera_name.c_str());
		if(camera == 0) {
			throw runtime_error("openCamera failed - No Camera with name '" + camera_name + "' found!");
		}
		stream = arv_camera_create_stream (camera, NULL, NULL);

		//TODO: Emit Signals nur anschalten, wenn auch ein Callback gesetzt wird...
		arv_stream_set_emit_signals (stream, TRUE);    

		//
		// Load the current settings of the camera
		//
		
		//Get Size of each frame from camera
		payload = arv_camera_get_payload (camera);
		
		//Get Width and Height of Frames
		arv_camera_get_region (camera, NULL, NULL, &width, &height);
		
		//Determine correct format of the frame
		format = arv_camera_get_pixel_format(camera);
	}

	void CameraAravis::startCapture() {
		cout << "Starting capturing ..." << endl;
		int retval = g_signal_connect (stream, "new-buffer", G_CALLBACK (aravisCameraCallback), this);
		arv_camera_start_acquisition(camera);
		cout << retval  << endl;
	}

	void CameraAravis::stopCapture() {
		cout << "Stopping capturing ..." << endl;
		arv_camera_stop_acquisition(camera);
	}

	void CameraAravis::prepareBuffer(const size_t bufferLen) {
		camera_buffer = new base::samples::frame::Frame[bufferLen];


		for (unsigned i = 0; i < bufferLen; ++i){
			camera_buffer[i].init(width,height,8, convertArvToFrameMode(format),128,payload);
			arv_stream_push_buffer (stream, arv_buffer_new (payload, camera_buffer[i].getImagePtr()));
		}
	}

	void CameraAravis::printBufferStatus() {
		gint input_length, output_length;
		arv_stream_get_n_buffers(stream, &input_length, &output_length);
		cout << "Output Queue Length: " << output_length << " Input Queue Length: " << input_length << endl;
	}
	
	bool CameraAravis::retrieveFrame(base::samples::frame::Frame &frame,const int timeout) {
		printBufferStatus();
		ArvBuffer* arv_buffer = arv_stream_pop_buffer(stream);
		if(arv_buffer != NULL) {
			if(arv_buffer->status == ARV_BUFFER_STATUS_SUCCESS) {
				pthread_mutex_lock(&buffer_counter_lock);
				buffer_counter--;
				pthread_mutex_unlock(&buffer_counter_lock);
				//Got valid frame
				cout << "retriveFrame (valid)" << endl;

				camera_buffer[current_frame].swap(frame);
				frame.setStatus(base::samples::frame::STATUS_VALID);
				if(camera_buffer[current_frame].getStatus() != base::samples::frame::STATUS_VALID) {
					//When the frame is invalid, initialize it
					camera_buffer[current_frame].init(width, height, 8, convertArvToFrameMode(format), 128, payload);
				}
				arv_stream_push_buffer(stream, arv_buffer_new(payload, camera_buffer[current_frame].getImagePtr()));

				current_frame = (current_frame + 1) % buffer_len;
				return true;
			} else {
				cout << "Wrong status of buffer: " << getBufferStatusString(arv_buffer->status) << endl;
				buffer_counter--;
				arv_stream_push_buffer(stream, arv_buffer_new(payload, camera_buffer[current_frame].getImagePtr()));
				current_frame = (current_frame + 1) % buffer_len;
				return false;
			}
		} else {
			cout << "Got Null pointer for buffer" << endl;
			return false;
		}
	}
	std::string CameraAravis::getBufferStatusString(ArvBufferStatus status) {
		switch(status) {
			case ARV_BUFFER_STATUS_SUCCESS:
				return "ARV_BUFFER_STATUS_SUCCESS";
			case ARV_BUFFER_STATUS_MISSING_PACKETS:
				return "ARV_BUFFER_STATUS_MISSING_PACKETS";
			case ARV_BUFFER_STATUS_CLEARED:
				return "ARV_BUFFER_STATUS_CLEARED";
			case ARV_BUFFER_STATUS_TIMEOUT:
				return "ARV_BUFFER_STATUS_TIMEOUT";
			case ARV_BUFFER_STATUS_WRONG_PACKET_ID:
				return "ARV_BUFFER_STATUS_WRONG_PACKET_ID";
			case ARV_BUFFER_STATUS_SIZE_MISMATCH:
				return "ARV_BUFFER_STATUS_SIZE_MISMATCH";
			case ARV_BUFFER_STATUS_FILLING:
				return "ARV_BUFFER_STATUS_FILLING";
			case ARV_BUFFER_STATUS_ABORTED:
				return "ARV_BUFFER_STATUS_ABORTED";

		}
	}

	base::samples::frame::frame_mode_t CameraAravis::convertArvToFrameMode(ArvPixelFormat format) {
		switch(format) {
			case ARV_PIXEL_FORMAT_BAYER_GB_8:
				return base::samples::frame::MODE_BAYER_GBRG;
			case ARV_PIXEL_FORMAT_MONO_8:
				return base::samples::frame::MODE_GRAYSCALE;
			default:
				throw runtime_error("Frame Format unknown!");
		}
	}

        bool CameraAravis::grab(const GrabMode mode, const int buffer_len) {
		if(camera == 0) {
			throw runtime_error("Camera not configured, please open one with openCamera() first!");
		}

		switch(mode) {
			case Continuously:
				this->buffer_len = buffer_len;
				prepareBuffer(buffer_len);
				startCapture();
				break;
			case Stop:
				stopCapture();
				break;
			default:
				throw runtime_error("Capture mode not supported!");
		}
		return true;
	}
       	bool CameraAravis::setAttrib(const int_attrib::CamAttrib attrib,const int value) {
		switch(attrib) {
			case int_attrib::ExposureValue:
				arv_camera_set_exposure_time(camera, value);
				break;
			case int_attrib::GainValue:
				arv_camera_set_gain(camera, value);
				break;
		}
	}
        int CameraAravis::getAttrib(const int_attrib::CamAttrib attrib) {
		switch(attrib) {
			case int_attrib::ExposureValue:
				return arv_camera_get_exposure_time(camera);
			case int_attrib::GainValue:
				return arv_camera_get_gain(camera);
		}
	}

	bool CameraAravis::setFrameSettings(  const base::samples::frame::frame_size_t size, 
                                          const base::samples::frame::frame_mode_t mode,
                                          const uint8_t color_depth,
                                          const bool resize_frames) {
		ArvPixelFormat targetPixelFormat;
		switch(mode) {
			case base::samples::frame::MODE_BAYER:
				targetPixelFormat = ARV_PIXEL_FORMAT_BAYER_GB_8;
				break;
			case base::samples::frame::MODE_GRAYSCALE:
				targetPixelFormat = ARV_PIXEL_FORMAT_MONO_8;
				break;

		}
		arv_camera_set_pixel_format(camera, targetPixelFormat);
		return true;
	}

	bool CameraAravis::setCallbackFcn(void (*pcallback_function)(const void* p),void *p) {
		cout << "Register Callback Fcn" << endl;
		callbackFcn = pcallback_function;
		callbackData = p;
		return true;
	}
	bool CameraAravis::isAttribAvail(const int_attrib::CamAttrib attrib) {
		if(attrib == int_attrib::ExposureValue || attrib == int_attrib::GainValue) {
			return true;
		}
		return false;
	}
	bool CameraAravis::isAttribAvail(const double_attrib::CamAttrib attrib) {
		return false;
	}
	bool CameraAravis::isAttribAvail(const str_attrib::CamAttrib attrib) {
		return false;
	}
	bool CameraAravis::isAttribAvail(const enum_attrib::CamAttrib attrib) {
		return false;
	}
        bool CameraAravis::close() {
		//TODO: Close Camera
		return true;
	}
	std::string CameraAravis::doDiagnose() {
		return "Here could be your super camera diagnose string...";
	}
	bool CameraAravis::isFrameAvailable() {
		//HACK: Eigentlich sollte der Task mit Callback benutzt werden, dann ist diese Methode eigentlich gar nicht notwendig, aber der camera_base Task erwartet
		//trotzdem das hier sinnvolle Werte zurückgegeben werden
		return buffer_counter > 0;
		
		/*
		int value = 0;
		if(-1 == sem_getvalue(&buffer_lock, &value)) {
			perror("sem_getvalue");
		}
		cout << "IsFrameAvailable: " << value << endl;
		return value > 0;*/
	}
}
