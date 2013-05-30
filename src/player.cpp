#include <cstdio>
#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <arv.h>

using namespace std;
using namespace cv;

int main(int argn, char** argv) {
	if(argn != 2) {
		cerr << "Bitte Logfile angegeben!" << endl;
		return -1;
	}

	string logfileName = string(argv[1]);

	FILE* logfile = fopen(logfileName.c_str(), "r");

	if(!logfile) {
		cerr << "Logfile konnte nicht geöffnet werden!" << endl;
		return -1;
	}

	//Create OpenCV Window
	namedWindow("Video");
	int32_t width, height, frame_id;
	int64_t timestamp_ns;
	int64_t last_timestamp = 0;
	ArvPixelFormat format;

	//Load Framestream
	while(!feof(logfile)) {
		fread(&width, sizeof(int32_t), 1, logfile);
		fread(&height, sizeof(int32_t), 1, logfile);
		fread(&frame_id, sizeof(int32_t), 1, logfile);
		fread(&timestamp_ns, sizeof(int64_t), 1, logfile);
		Mat image(Size(width, height), CV_8UC1);
		fread(image.data, width*height, 1, logfile);

		cout << "Read frame... (id=" << frame_id << ", Timestamp: " << timestamp_ns << ")" << endl;
		
		Mat converted;

		//cvtColor(image, converted, CV_BayerGB2RGB);

		imshow("Video", image);
		int64_t timeToWait = last_timestamp != 0 ? (timestamp_ns - last_timestamp)/1000000 : 1;
		last_timestamp = timestamp_ns;
		waitKey(timeToWait);
	}
}
