#include <cstdio>
#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <arv.h>
#include "videolog.hpp"
#include "frame_helper/AutoWhiteBalance.h"

using namespace std;
using namespace cv;

int main(int argn, char** argv) {
	if(argn != 2) {
		cerr << "Bitte Logfile angegeben!" << endl;
		return -1;
	}

	string logfileName = string(argv[1]);
	VideoLogfile logfile;
	logfile.open(logfileName, READER);

	//Create OpenCV Window
	namedWindow("Video");
	int64_t last_timestamp = 0;

	//Load Framestream
	while(true) {
		VideoFrame currentFrame;

		logfile >> currentFrame;

		if(logfile.eof()) 
			return 0;

		cout << "Read frame... (id=" << currentFrame.frame_id << ", Timestamp: " << currentFrame.timestamp_ns << ")" << endl;
		
		Mat converted;

		cvtColor(currentFrame.image, converted, CV_BayerGB2RGB);

		//Add auto white balance
		AutoWhiteBalancer* awb = AutoWhiteBalance::createAutoWhiteBalancer(converted);
		awb->applyCalibration(converted);
		delete awb;

		imshow("Video", converted);
		int64_t timeToWait = last_timestamp != 0 ? (currentFrame.timestamp_ns - last_timestamp)/1000000 : 1;
		last_timestamp = currentFrame.timestamp_ns;
		waitKey(timeToWait);
	}
}
