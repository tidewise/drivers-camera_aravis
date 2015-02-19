#include <iostream>
#include <highgui.h>
#include <arv.h>
#include <cstdio>
#include <unistd.h>
#include <cv.h>
#include <algorithm>

#include <camera_interface/BrightnessIndicator.h>
#include <camera_interface/ExposureController.h>
#include <camera_interface/AutoWhiteBalance.h>

using namespace std;
using namespace cv;

struct CamSettings {
	string name;
	double gain;
	double exposure;
	ArvPixelFormat pixel_format;
};

//const string CAMERA_NAME = "Aravis-GV01";
const CamSettings CAMERAS[] = {{"The Imaging Source Europe GmbH-42210449", 1.0, 1000, ARV_PIXEL_FORMAT_BAYER_GB_8},
				{"The Imaging Source Europe GmbH-29210317", 1.0, 1000, ARV_PIXEL_FORMAT_BAYER_GB_8},
				{"Aravis-GV01", 1.0, 10000, ARV_PIXEL_FORMAT_MONO_8}
				};
bool shouldExit = false;

FILE* openLogFile() {
	time_t rawtime;
	struct tm * timeinfo;

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	char buffer[255];
	strftime(buffer, 255, "video-%Y-%m-%d-%H%M%S.dat", timeinfo);
	FILE* fdesc = fopen(buffer, "w");
	if(!fdesc) {
		cerr << "Fehler beim öffnen der Logdatei..." << endl;
		exit(-1);
	}
	
	return fdesc;
}


void atExit(int signum) {
	shouldExit = true;
}

class NewExposureController {
	private:
		int min, max, tolerance;
		int lastExp;
		int calcNewValue(int measuredValue, int target) {
			if(measuredValue == 0) {
				return lastExp + 1000;
			}
			if(abs(measuredValue - target) > tolerance) { 
				double mx = lastExp * 1.0 / measuredValue;
				cout << "MX:" << mx << endl;
				return mx * target;
			} else {
				return lastExp;
			}
		}	
	public:
		NewExposureController(int _min, int _max, int _tolerance, int _currentExp) : tolerance(_tolerance), min(_min), max(_max), lastExp(_currentExp) {}
		int update(int measuredValue, int target) {
			int value = calcNewValue(measuredValue, target);
			value = std::min(max, value);
			value = std::max(min, value);
			lastExp = value;
			
			return value;
		}
};

int main(int argc, const char *argv[])
{
	//Kameraauswahl
	if(argc != 2) {
		cerr << "Bitte Kamera Nr. angeben! (1 oder 2)" << endl;
		return -1;
	}

	//GLib Initialisieren
	g_type_init();

	//Exit Handler
	signal(SIGINT, atExit);

	ArvCamera *camera;
	ArvStream *stream;
	unsigned int camera_sel = atoi(argv[1]);
	camera = arv_camera_new(CAMERAS[camera_sel].name.c_str());

	if(camera == 0) {
		cerr << "Konnte Kamera nicht finden" << endl;
		return -1;
	}

	
	stream = arv_camera_create_stream(camera, NULL, NULL);
	if(stream == 0) {
		cerr << "Konnte keinen Stream öffnen" << endl;
		return -1;
	}
	double minBound, maxBound;
	arv_camera_get_exposure_time_bounds(camera, &minBound, &maxBound);
	cout << "Min Exposure Time: " << minBound << endl;
	cout << "Max Exposure Time: " << maxBound << endl;

	arv_camera_set_pixel_format(camera, CAMERAS[camera_sel].pixel_format);
	arv_camera_set_gain(camera, CAMERAS[camera_sel].gain);
	arv_camera_set_exposure_time(camera, CAMERAS[camera_sel].exposure);
	int exposure = arv_camera_get_exposure_time(camera);
	cout << "Frame-Rate:" << arv_camera_get_frame_rate(camera) << endl;
	int payload = arv_camera_get_payload(camera); 

	for(int i=0;i<50;++i) {
		arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL)); 
	}

	//FILE* logfile = openLogFile();

	SimpleBrightnessIndicator sb;
	NewExposureController exposureController(100, 70000, 5, exposure); 
	arv_camera_start_acquisition(camera);

	//Poll changes
	ArvBuffer *arv_buffer = 0, *last_buffer = 0;
	namedWindow("Video");
	
	AutoWhiteBalancer *awb;
	arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Red");
	int camera_r_balance = arv_device_get_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw");
	arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Green");
	int camera_g_balance = arv_device_get_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw");
	arv_device_set_string_feature_value(arv_camera_get_device(camera), "BalanceRatioSelector", "Blue");
	int camera_b_balance = arv_device_get_integer_feature_value(arv_camera_get_device(camera), "BalanceRatioRaw");

	while(!shouldExit) {
		arv_buffer = arv_stream_pop_buffer(stream);
		if(arv_buffer != NULL) {
			if(arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS) {
				int width = arv_buffer_get_image_width(arv_buffer);
                                int height = arv_buffer_get_image_height(arv_buffer);
                                uint32_t frame_id = arv_buffer_get_frame_id(arv_buffer);
                                size_t size;
                                const void *data = arv_buffer_get_data(arv_buffer,&size);
				const Mat image(height, width, CV_8UC1, (void*)data);

				int brightness = sb.getBrightness(image);
				Mat converted;

				cvtColor(image, converted, CV_BayerGB2RGB);

				std::cout << "Exposure: " << exposure << std::endl;
				if(arv_buffer_get_frame_id(arv_buffer) % 4 == 0) {
					exposure = exposureController.update(brightness, 100);
					arv_camera_set_exposure_time(camera, exposure);

					awb = AutoWhiteBalance::createAutoWhiteBalancer(converted);
					std::cout << "AWB B: " << awb->offsetRight[0] << "(250)"<< std::endl;
					std::cout << "AWB G: " << awb->offsetRight[1] << std::endl;
					std::cout << "AWB R: " << awb->offsetRight[2] << std::endl;

					if(awb->offsetRight[0] < 250) {
						int err = 250 - awb->offsetRight[0];
						std::cout << "B corr: " << err << std::endl;
						camera_b_balance += err * 0.3;
					}
					if(awb->offsetRight[1] < 250) {
						int err = 250 - awb->offsetRight[1];
						std::cout << "G corr: " << err << std::endl;
						camera_g_balance += err * 0.3;
					}
					if(awb->offsetRight[2] < 250) {
						int err = 250 - awb->offsetRight[2];
						std::cout << "R corr: " << err << std::endl;
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

					std::cout << "Current Ratio B: " << camera_b_balance << std::endl;
					std::cout << "Current Ratio G: " << camera_g_balance << std::endl;
					std::cout << "Current Ratio R: " << camera_r_balance << std::endl;
				}
				
				resize(converted, converted, Size(0,0), 0.6, 0.6);
				imshow("Video", converted);
				waitKey(1);
			} else {
				cout << "Status bad..." << arv_buffer_get_status(arv_buffer) << endl;
			}
			arv_stream_push_buffer(stream, arv_buffer);
		} else {
			cout << "No data" << endl;
		}
	}
	cout << "Exiting" << endl;
	arv_camera_stop_acquisition(camera);
	g_object_unref(stream);

	//fclose(logfile);
	return 0;
}

