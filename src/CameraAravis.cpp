#include "CameraAravis.hpp"

#include <frame_helper/FrameHelper.h>
#include <exception>
#include <semaphore.h>

using namespace std;

namespace camera
{
    void aravisCameraCallback(ArvStream *stream, CameraAravis *driver) 
    {
        pthread_mutex_lock(&(driver->buffer_counter_lock));
        driver->buffer_counter++;
        pthread_mutex_unlock(&(driver->buffer_counter_lock));
        if(driver->callbackFcn != 0) 
            driver->callbackFcn(driver->callbackData);
    }
	
    void controlLostCallback (CameraAravis *driver) 
    {
        std::cout << "ERROR: The connection was lost. Try to plug the camera and re-run the task." << std::endl;
        if(driver->errorCallbackFcn != 0)
            driver->errorCallbackFcn(driver->callbackData);
    }

    CameraAravis::CameraAravis():
        camera(NULL),
        stream(NULL),
        current_frame(0),
        buffer_counter(0),
        callbackFcn(NULL),
        errorCallbackFcn(NULL)
    {
        pthread_mutex_init(&buffer_counter_lock, NULL);
    }

    CameraAravis::~CameraAravis()
    {
        if(stream)
            g_object_unref(stream);
        if(camera)
            g_object_unref(camera);
    }

    void CameraAravis::openCamera(std::string camera_name)
    {
        if(camera)
            throw runtime_error("openCamera failed - camera is already openend");
        camera = arv_camera_new(camera_name.c_str());
        if(camera == 0)
            throw runtime_error("openCamera failed - No Camera with name '" + camera_name + "' found!");
        stream = arv_camera_create_stream (camera, NULL, NULL);
        if(stream == 0)
            throw runtime_error("openCamera failed - Cannot create stream");

        arv_stream_set_emit_signals (stream, TRUE);

    }

    void CameraAravis::startCapture()
    {
        if(!camera)
            throw std::runtime_error("CameraAravis: camera is not opened!");
        arv_camera_start_acquisition(camera);

        callback_handler = g_signal_connect (stream, "new-buffer", G_CALLBACK (aravisCameraCallback), this);
        if(0 >= callback_handler)
            throw std::runtime_error("CameraAravis: cannot connect callback!");

        error_callback_handler = g_signal_connect(arv_camera_get_device(camera), "control-lost", G_CALLBACK (controlLostCallback), NULL);
        if(0 >=error_callback_handler)
            throw std::runtime_error("CameraAravis: cannot connect error callback!");
    }

    void CameraAravis::stopCapture()
    {
        if(!camera)
            return;
        g_signal_handler_disconnect (stream,callback_handler);
        g_signal_handler_disconnect (arv_camera_get_device(camera),error_callback_handler);
        arv_camera_stop_acquisition(camera);
    }

    void CameraAravis::prepareBuffer(const size_t bufferLen)
    {
        // Load the current settings of the camera
        //Get Size of each frame from camera
        unsigned int payload = arv_camera_get_payload (camera);

        //Get Width and Height of Frame
        int width, height;
        arv_camera_get_region (camera, NULL, NULL, &width, &height);
        //Determine correct format of the frame
        ArvPixelFormat format = arv_camera_get_pixel_format(camera);

        camera_buffer.resize(bufferLen);
        for (unsigned int i = 0; i < bufferLen; ++i)
        {
            camera_buffer[i].init(width,height,8,convertArvToFrameMode(format),-1,payload);
            arv_stream_push_buffer(stream, arv_buffer_new(payload, camera_buffer[i].getImagePtr()));
        }
    }

    void CameraAravis::printBufferStatus()
    {
        gint input_length, output_length;
        arv_stream_get_n_buffers(stream, &input_length, &output_length);
    }
	
    bool CameraAravis::retrieveFrame(base::samples::frame::Frame &frame,const int timeout)
    {
        ArvBuffer* arv_buffer = arv_stream_pop_buffer(stream);
        if(arv_buffer == NULL)
            return false; // no image

        pthread_mutex_lock(&buffer_counter_lock);
        if(buffer_counter > 0)
            buffer_counter--;
        pthread_mutex_unlock(&buffer_counter_lock);

        bool bok = true;
        if(arv_buffer_get_status(arv_buffer) != ARV_BUFFER_STATUS_SUCCESS)
        {
            frame.time = base::Time::now();
            frame.received_time = frame.time;
            frame.setStatus(base::samples::frame::STATUS_INVALID);
            arv_stream_push_buffer(stream,arv_buffer);
            bok = false;
        }
        else
        {
            //Got valid frame
            base::samples::frame::Frame &new_frame = camera_buffer[current_frame];
            new_frame.swap(frame);
            new_frame.init(frame.size.width,frame.size.height,8,frame.frame_mode,-1); // ensure right size
            frame.setStatus(base::samples::frame::STATUS_VALID);
            frame.time = base::Time::fromMicroseconds(arv_buffer_get_timestamp(arv_buffer)*0.001);
            frame.received_time = base::Time::now();
            //we have to create a new buffer because pointer has changed 
            g_object_unref(arv_buffer);
            arv_buffer = NULL;
            arv_stream_push_buffer(stream, arv_buffer_new(new_frame.image.size(),new_frame.getImagePtr()));
        }
        current_frame = (current_frame+1)%camera_buffer.size();
        return bok;
    }

    std::string CameraAravis::getBufferStatusString(ArvBufferStatus status) 
    {
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

    base::samples::frame::frame_mode_t CameraAravis::convertArvToFrameMode(ArvPixelFormat format) 
    {
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


    bool CameraAravis::grab(const GrabMode mode, const int buffer_len)
    {
        if(camera == 0)
            throw runtime_error("Camera not configured, please open one with openCamera() first!");

        switch(mode)
        {
        case Continuously:
            prepareBuffer(buffer_len);
            arv_camera_set_acquisition_mode (camera, ARV_ACQUISITION_MODE_CONTINUOUS);
            startCapture();
            break;
            /*case SingleFrame:
              arv_camera_set_acquisition_mode (camera, ARV_ACQUISITION_MODE_SINGLE_FRAME);
              prepareBuffer(1);				
              startCapture();
              break;*/
        case Stop:
            stopCapture();
            break;
        default:
            throw runtime_error("Capture mode not supported!");
        }
        return true;
    }

	ArvPixelFormat CameraAravis::getBayerFormat (){
		guint n;
		gint64 * available_formats = arv_camera_get_available_pixel_formats(camera, &n);

		for (guint x = 0; x<n; ++x){
			switch (available_formats[x]){
				case ARV_PIXEL_FORMAT_BAYER_GR_8:
					return ARV_PIXEL_FORMAT_BAYER_GR_8;
				case ARV_PIXEL_FORMAT_BAYER_RG_8:
					return ARV_PIXEL_FORMAT_BAYER_RG_8;
				case ARV_PIXEL_FORMAT_BAYER_GB_8:
					return ARV_PIXEL_FORMAT_BAYER_GB_8;
				case ARV_PIXEL_FORMAT_BAYER_BG_8:
					return ARV_PIXEL_FORMAT_BAYER_BG_8;				
			}
 		}			
		//It is executed if any bayer mode was found
		throw runtime_error("Bayer format is not supported by the camera");
	}

       	bool CameraAravis::setAttrib(const int_attrib::CamAttrib attrib,const int value) {
		switch(attrib) {
			case int_attrib::ExposureValue:{
				arv_camera_set_exposure_time_auto(camera, ARV_AUTO_OFF);				
				arv_camera_set_exposure_time(camera, value);
				double exposure = arv_camera_get_exposure_time(camera);
				if (exposure != value) throw runtime_error("The attribute 'exposure' could not be set");		
				break;
			}
			case int_attrib::GainValue:{
				arv_camera_set_gain(camera, value);
				int gain = arv_camera_get_gain(camera);	
				if (gain != value) throw runtime_error("The attribute 'gain' could not be set");
				break;
			}
			case int_attrib::RegionX:{
				int region_x, region_y,width,height;
				arv_camera_get_region (camera, &region_x, &region_y, &width, &height);
				arv_camera_set_region (camera, value, region_y, width, height);
				arv_camera_get_region (camera, &region_x, &region_y, &width, &height);
				if (region_x != value) throw runtime_error("The attribute region_x could not be set.");
				
				int sensor_width, sensor_height;
				arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height); 

				if ((region_x + width)> sensor_width){
				 throw runtime_error("The attribute region_x could not be set. The 'region_x' plus 'width' must be less or equal the width supported by the camera");
				}
				break;

			}
			case int_attrib::RegionY:{
				int region_x, region_y,width,height;
				arv_camera_get_region (camera, &region_x, &region_y, &width, &height);
				arv_camera_set_region (camera, region_x, value, width, height);
				arv_camera_get_region (camera, &region_x, &region_y, &width, &height);
				
				if (region_y != value) throw runtime_error("The attribute region_y could not be set.");

				int sensor_width, sensor_height;
				arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height); 

				if ((region_y + height) > sensor_height) 
                                    throw runtime_error("The attribute region_y could not be set. The 'region_y' plus 'height' must be less or equal the width supported by the camera");
				break;

			}
			case int_attrib::BinningX:{
				int binning_x, binning_y;		
				arv_camera_get_binning (camera, &binning_x, &binning_y);
				arv_camera_set_binning (camera, value, binning_y);
				arv_camera_get_binning (camera, &binning_x, &binning_y);

				// check the return value of the binning.
				// Everything is ok when it matches however,
				// when the set value is requested for 1, the
				// result may also be 0.  This last condition
				// is an assumption based on what the previous
				// version of what this code seemed to want to
				// do.
                                if (binning_x != 0 && value != 1 && binning_x != value)
                                        throw runtime_error("Camera does not support binning.");
				break;
			}

			case int_attrib::BinningY:{
				int binning_x, binning_y;		
				arv_camera_get_binning (camera, &binning_x, &binning_y);
				arv_camera_set_binning (camera, binning_x, value);
				arv_camera_get_binning (camera, &binning_x, &binning_y);

                                // see check of binning_x for explanations
                                if (binning_y != 0 && value != 1 && binning_y != value)
                                    throw runtime_error("Camera does not support binning.");
			}
			default:
				break;
		}
		return true;
	}

       	bool CameraAravis::setAttrib(const double_attrib::CamAttrib attrib,const double value) {
		switch(attrib) {
                    case double_attrib::FrameRate:{
		            arv_camera_stop_acquisition(camera); 
                            arv_camera_set_frame_rate(camera, value);
                            double fps = arv_camera_get_frame_rate(camera);
			    if (fabs(fps - value) > 0.01) throw runtime_error("The attribute 'FrameRate' could not be set");
			    arv_camera_start_acquisition(camera);
                            return true;
		    }
                    default:
                            return false;
		}
	}

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

	bool CameraAravis::setFrameSettings(  const base::samples::frame::frame_size_t size, 
                                          const base::samples::frame::frame_mode_t mode,
                                          const uint8_t color_depth,
                                          const bool resize_frames) {
		ArvPixelFormat targetPixelFormat;
		switch(mode) {
			case base::samples::frame::MODE_BAYER:
			    targetPixelFormat = getBayerFormat();
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

		int sensor_width, sensor_height;
		arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height); 
		if ((size.width > sensor_width) || (size.height > sensor_height)){
		throw runtime_error("This size is not supported by the camera. Please check the maximum resolution of the camera");		
		}

		if ((size.width%8 !=0) || (size.height%8 !=0)){
		throw runtime_error("The size of the frame must be dividable by 8");
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

	bool CameraAravis::setErrorCallbackFcn(void (*pcallback_function)(const void* p),void *p) {
		errorCallbackFcn = pcallback_function;
		errorCallbackData = p;
		return true;
	}

	bool CameraAravis::isAttribAvail(const int_attrib::CamAttrib attrib) {
		switch (attrib) {
			case int_attrib::ExposureValue:
				return true;
			case int_attrib::GainValue:
				return true;
			case int_attrib::RegionX: 
				return true;
			case int_attrib::RegionY:
				return true;
			case int_attrib::BinningX:
				return true;		
			case int_attrib::BinningY:
				return true;
			default: 
				return false;			
		}		
	}

	bool CameraAravis::isAttribAvail(const double_attrib::CamAttrib attrib) {
                return  attrib == double_attrib::FrameRate;
	}

	bool CameraAravis::isAttribAvail(const str_attrib::CamAttrib attrib) {
		return false;
	}

	bool CameraAravis::isAttribAvail(const enum_attrib::CamAttrib attrib) {
		switch(attrib){
			case enum_attrib::ExposureModeToManual:
				return true;			
			case enum_attrib::ExposureModeToAutoOnce:
				return true;
			case enum_attrib::ExposureModeToAuto:
				return true;
			case enum_attrib::WhitebalModeToAuto:
				return true;			
			case enum_attrib::WhitebalModeToManual:
				return true;
			case enum_attrib::GainModeToManual:
				return true;
			case enum_attrib::GainModeToAutoOnce:
				return true;
			case enum_attrib::GainModeToAuto:
				return true;
			default:
				return false;
		}
	}

	bool CameraAravis::setAttrib(const enum_attrib::CamAttrib attrib) {
		switch(attrib) {
			case enum_attrib::ExposureModeToManual:
				arv_camera_set_exposure_time_auto(camera, ARV_AUTO_OFF);
				return true;
			case enum_attrib::ExposureModeToAutoOnce:
				arv_camera_set_exposure_time_auto(camera, ARV_AUTO_ONCE);
				return true;
			case enum_attrib::ExposureModeToAuto:
				arv_camera_set_exposure_time_auto(camera, ARV_AUTO_CONTINUOUS);
				return true;
			case enum_attrib::WhitebalModeToManual:
				return true;
			case enum_attrib::WhitebalModeToAuto:
				return true;
			case enum_attrib::GainModeToManual:
				arv_camera_set_gain_auto (camera, ARV_AUTO_OFF);
				return true;
			case enum_attrib::GainModeToAutoOnce:
				arv_camera_set_gain_auto (camera, ARV_AUTO_ONCE);
				return true;
			case enum_attrib::GainModeToAuto:
				arv_camera_set_gain_auto (camera, ARV_AUTO_CONTINUOUS);
				return true;
			default:
				return false;
		}
	}

	bool CameraAravis::isAttribSet(const enum_attrib::CamAttrib attrib) {
		switch(attrib) {
			case enum_attrib::ExposureModeToManual:
				if (arv_camera_get_exposure_time_auto(camera) == ARV_AUTO_OFF) return true;
				else return false;
			case enum_attrib::ExposureModeToAutoOnce:{
				if (arv_camera_get_exposure_time_auto(camera) == ARV_AUTO_ONCE) return true;
				else return false;
			}
			case enum_attrib::ExposureModeToAuto:{
				if (arv_camera_get_exposure_time_auto(camera) == ARV_AUTO_CONTINUOUS) return true;
				else return false;
			}
			case enum_attrib::WhitebalModeToManual:
                                break;
			case enum_attrib::WhitebalModeToAuto:
                                break;
			case enum_attrib::GainModeToManual:{
				ArvAuto status = arv_camera_get_gain_auto(camera);				
				if (status == ARV_AUTO_OFF) return true;
				else return false;
			}
			case enum_attrib::GainModeToAutoOnce:{
				ArvAuto status = arv_camera_get_gain_auto(camera);
				if (status == ARV_AUTO_ONCE) return true;
				else return false;
			}
			case enum_attrib::GainModeToAuto:{
				ArvAuto status = arv_camera_get_gain_auto(camera);
				if (status == ARV_AUTO_CONTINUOUS) return true;
				else return false;
			}
			default:
				return false;
		}
                return false;
	}

        bool CameraAravis::close() {
		//TODO: Close Camera
		return true;
	}
	std::string CameraAravis::doDiagnose() {
		return "Here could be your super camera diagnose string...";
	}

        bool CameraAravis::isFrameAvailable() 
        {
            //HACK: Eigentlich sollte der Task mit Callback benutzt werden, dann ist diese Methode eigentlich gar nicht notwendig, aber der camera_base Task erwartet
            //trotzdem das hier sinnvolle Werte zurückgegeben werden
            return buffer_counter > 0;
        }
}
