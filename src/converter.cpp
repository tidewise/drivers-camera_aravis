#include <stdio.h>
#include <sstream>
#include <cv.h>
#include <highgui.h>
#include "videolog.hpp"

using namespace std;
using namespace cv;

int main(int argn, const char *argv[])
{
	if(argn != 4) {
		cerr << "Syntax: aravis-test-converter <Logfile> <Output AVI> <FPS>" << endl;
		return -1;
	}

	string logfileName = string(argv[1]);
	string outputfileName = string(argv[2]);
	int fps = atoi(argv[3]);
	
	VideoLogfile logfile;
	logfile.open(logfileName, READER);
	VideoFrame currentFrame;
	logfile >> currentFrame;

	VideoWriter output(outputfileName, CV_FOURCC('D','I','V','X'), fps, currentFrame.image.size(), true);

	Mat converted;
	cvtColor(currentFrame.image, converted, CV_BayerGB2RGB);

	output << converted; 

	while(true) {
		logfile >> currentFrame;

		if(logfile.eof()) 
			return 0;

		cvtColor(currentFrame.image, converted, CV_BayerGB2RGB);
		output << converted; 
	}
}
