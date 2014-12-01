#include <iostream>
#include <highgui.h>
#include <arv.h>
#include <cstdio>
#include <unistd.h>
#include <cv.h>

#include <camera_interface/BrightnessIndicator.h>
#include <camera_interface/ExposureController.h>

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
        size_t size;
        const void *data = arv_buffer_get_data(frame,&size);
        gint width = arv_buffer_get_image_width(frame);
        gint height = arv_buffer_get_image_height(frame);
        guint32 frame_id = arv_buffer_get_frame_id(frame); 
        guint64 timesatmp_ns = arv_buffer_get_timestamp(frame);
	int ret =  fwrite(&width,       sizeof(width), 1, fhandle);
	int ret2 = fwrite(&height,      sizeof(height), 1, fhandle);
	int ret3 = fwrite(&frame_id,    sizeof(frame_id), 1, fhandle);
	int ret4 = fwrite(&timesatmp_ns,sizeof(timesatmp_ns), 1, fhandle);
	int ret5 = fwrite(data, size, 1, fhandle);

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

	FILE* logfile = openLogFile();

	SimpleBrightnessIndicator sb;
	ExposureController exposureController(100, 70000, 5, exposure);
	arv_camera_start_acquisition(camera);

	//Poll changes
	ArvBuffer *arv_buffer = 0, *last_buffer = 0;
	while(!shouldExit) {
		arv_buffer = arv_stream_pop_buffer(stream);
		if(arv_buffer != NULL) {
			if(arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS) {
				int width = arv_buffer_get_image_width(arv_buffer);
                                int height = arv_buffer_get_image_height(arv_buffer);
                                size_t size;
                                const void *data = arv_buffer_get_data(arv_buffer,&size);
				const Mat image(height, width, CV_8UC1, (void*)data);

				int brightness = sb.getBrightness(image);

				exposure = exposureController.update(brightness, 100);
				std::cout << "Exposure: " << exposure << std::endl;
				arv_camera_set_exposure_time(camera, exposure);

				//Check frame format
				saveFrame(logfile, arv_buffer); 
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

	fclose(logfile);
	return 0;
}

