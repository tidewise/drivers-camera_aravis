#include <iostream>
#include <highgui.h>
#include <arv.h>
#include <cstdio>
#include <unistd.h>
#include <cv.h>

#include <camera_interface/BrightnessIndicator.h>
#include <camera_interface/ExposureController.h>
#include <motor_controller/PID.hpp>

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

void saveFrame(FILE* fhandle, ArvBuffer* frame) {
	int ret = fwrite(&(frame->width), sizeof(frame->width), 1, fhandle);
	int ret2 = fwrite(&(frame->height), sizeof(frame->height), 1, fhandle);
	int ret3 = fwrite(&(frame->frame_id), sizeof(frame->frame_id), 1, fhandle);
	int ret4 = fwrite(&(frame->timestamp_ns), sizeof(frame->timestamp_ns), 1, fhandle);
	int ret5 = fwrite(frame->data, frame->size, 1, fhandle);

	if(!(ret && ret2 && ret3 && ret4 && ret5)) {
		throw runtime_error("Error on write!");
	}
}

void atExit(int signum) {
	shouldExit = true;
}

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

	std::vector<WeightedRect> regions;
	regions.push_back(WeightedRect(-1.0, -1.0, 1.0, 1.0, 1));
	regions.push_back(WeightedRect(-0.3, -0.3, 0.3, 0.3, 3)); 
	WeightedBoxesBrightnessIndicator sb(regions);
	arv_camera_start_acquisition(camera);

	//Poll changes
	ArvBuffer *arv_buffer = 0, *last_buffer = 0;

	//PID
	motor_controller::PID pid; 
	//pid.setParallelCoefficients(1, 5, 100, 50, 2, 1, -1, 100, 100000);
	pid.setParallelCoefficients(1, 2, 10, 5, 2, 1, -1, 100, 100000);
	int threshold = 0;
	while(!shouldExit) {
		arv_buffer = arv_stream_pop_buffer(stream);
		if(arv_buffer != NULL) {
			if(arv_buffer->status == ARV_BUFFER_STATUS_SUCCESS) {
				int width = arv_buffer->width, height = arv_buffer->height;

				Mat image(height, width, CV_8UC1, arv_buffer->data);
				Mat converted;

				cvtColor(image, converted, CV_BayerGB2RGB);
				imshow("Video", converted);
				waitKey(1);

				int brightness = sb.getBrightness(image);
				cout << "Got image "  << arv_buffer->frame_id << " Brightness: " << brightness << endl;
				if(threshold == 0 || brightness > 100) {
					cout << "updating" << endl;
					exposure = pid.update(brightness, 100);
					if(exposure == 100000) {
						threshold = brightness;
					} else {
						threshold = 0;
					}
				}
				std::cout << "Exposure: " << exposure << std::endl;
				arv_camera_set_exposure_time(camera, exposure);

				//Check frame format
				//saveFrame(logfile, arv_buffer); 
			} else {
				cout << "Status bad..." << arv_buffer->status << endl;
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

