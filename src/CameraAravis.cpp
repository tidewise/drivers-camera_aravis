#include "CameraAravis.hpp"

#include <camera_interface/AutoWhiteBalance.h>
#include <frame_helper/FrameHelper.h>

#include <exception>
#include <semaphore.h>




using namespace std;

namespace camera
{
	void aravisCameraCallback(ArvStream *stream, CameraAravis *driver) {
		pthread_mutex_lock(&(driver->buffer_counter_lock));
		driver->buffer_counter++;
		pthread_mutex_unlock(&(driver->buffer_counter_lock));
		if(driver->callbackFcn != 0) {
			driver->callbackFcn(driver->callbackData);
		}
	}
	
	void controlLostCallback (CameraAravis *driver) {
		
		std::cout << "Control lost" << std::endl;
		driver->cancel = true;
		throw runtime_error("The connection was lost");
	}
	// implement new callback for control-lost
        // cache error (use mutex before setting variable)
        // call driver callback
        // throw error in retrieveFrame

	CameraAravis::CameraAravis() {

		path = strcat(getenv("AUTOPROJ_CURRENT_ROOT"),"/drivers/orogen/camera_aravis/scripts");
		fLS::FLAGS_log_dir = path;
		google::InitGoogleLogging("camera_aravis");

		g_type_init();

		camera = 0;
		stream = 0;
		current_frame = 0;
		buffer_len = 0;
		callbackFcn = 0;
		buffer_counter = 0;
		currentExposure = -1;
		exposureFrameCounter = 0;
		autoWhitebalance = false;
		autoExposure = false;
		pthread_mutex_init(&buffer_counter_lock, NULL);
	}

	CameraAravis::~CameraAravis() {
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
		
		cout << "width: " << width << endl;
		cout << "height: " << height << endl;
		cout << "payload: " << payload << endl;

		//Determine correct format of the frame
		format = arv_camera_get_pixel_format(camera);
		
		currentExposure = arv_camera_get_exposure_time(camera);
		exposureController.reset(new ExposureController(100, 70000, 5, currentExposure));
	}

	void CameraAravis::startCapture() {
		arv_camera_start_acquisition(camera);
		int retval = g_signal_connect (stream, "new-buffer", G_CALLBACK (aravisCameraCallback), this);
		g_signal_connect (arv_camera_get_device (camera), "control-lost", G_CALLBACK (controlLostCallback), NULL);
             
	}

	void CameraAravis::stopCapture() {
		arv_camera_stop_acquisition(camera);
               // disconntect control-lost
               // disconntect new-buffer
	}

	void CameraAravis::prepareBuffer(const size_t bufferLen) {

		camera_buffer.resize(bufferLen);
		for (unsigned i = 0; i < bufferLen; ++i){
			camera_buffer[i].init(width,height,8, convertArvToFrameMode(format),128,payload);
			arv_stream_push_buffer (stream, arv_buffer_new (payload, camera_buffer[i].getImagePtr()));
		}
	}

	void CameraAravis::printBufferStatus() {
		gint input_length, output_length;
		arv_stream_get_n_buffers(stream, &input_length, &output_length);
	}
	
	bool CameraAravis::retrieveFrame(base::samples::frame::Frame &frame,const int timeout) {
		// cehck if error was chached and throw if yes
		printBufferStatus();
		ArvBuffer* arv_buffer = arv_stream_pop_buffer(stream);
		if(arv_buffer != NULL) {
			if(arv_buffer->status == ARV_BUFFER_STATUS_SUCCESS) {
				pthread_mutex_lock(&buffer_counter_lock);
				buffer_counter--;
				pthread_mutex_unlock(&buffer_counter_lock);
				//Got valid frame

				camera_buffer[current_frame].swap(frame);
				frame.setStatus(base::samples::frame::STATUS_VALID);
				if(camera_buffer[current_frame].getStatus() != base::samples::frame::STATUS_VALID) {
					//When the frame is invalid, initialize it
					camera_buffer[current_frame].init(width, height, 8, convertArvToFrameMode(format), 128, payload);
				}
				arv_stream_push_buffer(stream, arv_buffer_new(payload, camera_buffer[current_frame].getImagePtr()));

				current_frame = (current_frame + 1) % buffer_len;

				//AutoWhitebalance if desired
				if(autoWhitebalance && arv_buffer->pixel_format == ARV_PIXEL_FORMAT_BAYER_GB_8) {
					base::samples::frame::Frame convertedFrame(height, width, 8, base::samples::frame::MODE_RGB);
					cv::Mat image(height, width, CV_8UC1, arv_buffer->data);
					cv::Mat converted = frame_helper::FrameHelper::convertToCvMat(convertedFrame);
					cv::cvtColor(image, converted, CV_BayerGB2RGB);
					
					AutoWhiteBalancer* awb = AutoWhiteBalance::createAutoWhiteBalancer(converted);

					if(awb->offsetRight[0] < 250) {
						int err = 250 - awb->offsetRight[0];
						camera_b_balance += err * 0.3;
					}
					if(awb->offsetRight[1] < 250) {
						int err = 250 - awb->offsetRight[1];
						camera_g_balance += err * 0.3;
					}
					if(awb->offsetRight[2] < 250) {
						int err = 250 - awb->offsetRight[2];
						camera_r_balance += err * 0.3;
					}

					double norm = sqrt((camera_b_balance * camera_b_balance) + (camera_r_balance + camera_r_balance) + (camera_g_balance * camera_g_balance));
					camera_b_balance = (camera_b_balance / norm) * 255;
					camera_r_balance = (camera_r_balance / norm) * 255;
					camera_g_balance = (camera_g_balance / norm) * 255;
					arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Red");
					arv_device_set_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw", camera_r_balance);
					arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Green");
					arv_device_set_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw", camera_g_balance);
					arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Blue");
					arv_device_set_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw", camera_b_balance);

				}
				// Remove autoExposure 


				//Adjust exposure if desired
				//Only use every third frame otherwise the controller is not stable because of the
				//async behaviour of arv_camera_set_exposure_time
				if(autoExposure && (exposureFrameCounter % 3 == 0)) {
					cv::Mat image(height, width, CV_8UC1, arv_buffer->data);
					int brightness = brightnessIndicator.getBrightness(image);
					currentExposure = exposureController->update(brightness, 100);
					arv_camera_set_exposure_time(camera, currentExposure);
				}
				++exposureFrameCounter;
                                frame.time = base::Time::now();
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
			default:
				throw runtime_error("This value for ArvBufferStatus is unknown!");

		}
	}

	base::samples::frame::frame_mode_t CameraAravis::convertArvToFrameMode(ArvPixelFormat format) {
		switch(format) {
			case ARV_PIXEL_FORMAT_MONO_8:
				return base::samples::frame::MODE_GRAYSCALE;
			case ARV_PIXEL_FORMAT_RGB_8_PACKED:
			        return base::samples::frame::MODE_RGB;
			case ARV_PIXEL_FORMAT_BGR_8_PACKED:
				return base::samples::frame::MODE_BGR;
			case ARV_PIXEL_FORMAT_BAYER_GR_8:
				return base::samples::frame::MODE_BAYER_GRBG;
			case ARV_PIXEL_FORMAT_BAYER_RG_8:
				return base::samples::frame::MODE_BAYER_RGGB;
			case ARV_PIXEL_FORMAT_BAYER_GB_8:
				return base::samples::frame::MODE_BAYER_GBRG;
			case ARV_PIXEL_FORMAT_BAYER_BG_8:
				return base::samples::frame::MODE_BAYER_BGGR;
			default:
				throw runtime_error("Frame Format unknown!");
		}
	}

       // implement single shot (defined in camera interface)
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

        // implement missing attributes (look into camera_interface CamTypes.h)
       	bool CameraAravis::setAttrib(const int_attrib::CamAttrib attrib,const int value) {
		switch(attrib) {
			case int_attrib::ExposureValue:
				arv_camera_set_exposure_time(camera, value);
				return true;
			case int_attrib::GainValue:
				arv_camera_set_gain(camera, value);
				return true;
			default:
				return false;
		}
	}

        // implement missing attributes (look into camera_interface)
       	bool CameraAravis::setAttrib(const double_attrib::CamAttrib attrib,const double value) {
		switch(attrib) {
                    case double_attrib::FrameRate:
		            arv_camera_stop_acquisition(camera);
                            arv_camera_set_frame_rate(camera, value);
                            arv_camera_start_acquisition(camera);
                            return true;
                    default:
                            return false;
		}
	}

        // implement missing attributes (look into camera_interface)
        int CameraAravis::getAttrib(const int_attrib::CamAttrib attrib) {
		switch(attrib) {
			case int_attrib::ExposureValue:
				return arv_camera_get_exposure_time(camera);
			case int_attrib::GainValue:
				return arv_camera_get_gain(camera);
			default:
				throw runtime_error("The attribute is not supported by the camera!");
		}
	}

        // add support BAYER (check which bayer pattern is supported by the camera and set it)
	bool CameraAravis::setFrameSettings(  const base::samples::frame::frame_size_t size, 
                                          const base::samples::frame::frame_mode_t mode,
                                          const uint8_t color_depth,
                                          const bool resize_frames) {
		ArvPixelFormat targetPixelFormat;
		switch(mode) {
			case base::samples::frame::MODE_BAYER:
				//TODO: ask camera which bayer pattern is supported and set it
				break;
			case base::samples::frame::MODE_GRAYSCALE:
			    targetPixelFormat = ARV_PIXEL_FORMAT_MONO_8;
			    break;
			case base::samples::frame::MODE_RGB:
			    targetPixelFormat = ARV_PIXEL_FORMAT_RGB_8_PACKED;
			    break;
			case base::samples::frame::MODE_BGR:
			    targetPixelFormat = ARV_PIXEL_FORMAT_BGR_8_PACKED;
			    break;
			case base::samples::frame::MODE_BAYER_GRBG:
			    targetPixelFormat = ARV_PIXEL_FORMAT_BAYER_GR_8;
			    break;
			case base::samples::frame::MODE_BAYER_RGGB:
			    targetPixelFormat = ARV_PIXEL_FORMAT_BAYER_RG_8;
			    break;
			case base::samples::frame::MODE_BAYER_GBRG:
			    targetPixelFormat = ARV_PIXEL_FORMAT_BAYER_GB_8;
			    break;
			case base::samples::frame::MODE_BAYER_BGGR:
			    targetPixelFormat = ARV_PIXEL_FORMAT_BAYER_BG_8;
			    break;
			default:
				throw runtime_error("This pixel format is not supported");
				break;

		}
		arv_camera_set_region (camera, 0, 0, size.width, size.height);
		arv_camera_set_pixel_format(camera, targetPixelFormat);
		return true;
	}

	bool CameraAravis::setCallbackFcn(void (*pcallback_function)(const void* p),void *p) {
		callbackFcn = pcallback_function;
		callbackData = p;
		return true;
	}

        // implement missing attributes (look into camera_interface)
	bool CameraAravis::isAttribAvail(const int_attrib::CamAttrib attrib) {
		if(attrib == int_attrib::ExposureValue || attrib == int_attrib::GainValue) {
			return true;
		}
		return false;
	}

        // implement missing attributes (look into camera_interface)
	bool CameraAravis::isAttribAvail(const double_attrib::CamAttrib attrib) {
                return  attrib == double_attrib::FrameRate;
	}

        // implement missing attributes (look into camera_interface)
	bool CameraAravis::isAttribAvail(const str_attrib::CamAttrib attrib) {
		return false;
	}

        // implement missing attributes (look into camera_interface)
	bool CameraAravis::isAttribAvail(const enum_attrib::CamAttrib attrib) {
		return (
                            attrib == enum_attrib::ExposureModeToManual ||
			    attrib == enum_attrib::ExposureModeToAuto ||
			    attrib == enum_attrib::WhitebalModeToAuto ||
			    attrib == enum_attrib::WhitebalModeToManual
                            );
	}

        // implement missing attributes (look into camera_interface)
	bool CameraAravis::setAttrib(const enum_attrib::CamAttrib attrib) {
		switch(attrib) {
			case enum_attrib::ExposureModeToManual:
				autoExposure = false;
				return true;
			case enum_attrib::ExposureModeToAuto:
				autoExposure = true;
				return true;
			case enum_attrib::WhitebalModeToManual:
				autoWhitebalance = false;
				return true;
			case enum_attrib::WhitebalModeToAuto:
				autoWhitebalance = true;
				arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Red");
				camera_r_balance = arv_device_get_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw");
				arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Green");
				camera_g_balance = arv_device_get_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw");
				arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Blue");
				camera_b_balance = arv_device_get_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw");
				return true;
			default:
				return false;
		}
	}

        // implement missing attributes (look into camera_interface)
	bool CameraAravis::isAttribSet(const enum_attrib::CamAttrib attrib) {
		switch(attrib) {
			case enum_attrib::ExposureModeToManual:
				return !autoExposure;
			case enum_attrib::ExposureModeToAuto:
				return autoExposure;
			case enum_attrib::WhitebalModeToManual:
				return !autoWhitebalance;
			case enum_attrib::WhitebalModeToAuto:
				return autoWhitebalance;
			default:
				return false;
		}
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
