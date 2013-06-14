#include "videolog.hpp"
#include <exception>
#include <arv.h>

using namespace std;
using namespace cv;

void VideoLogfile::open(std::string filename, VideoLogMode mode) {
	this->mode = mode;
	switch(mode) {
		case WRITER:
			logfile = fopen(filename.c_str(), "w");
			break;
		case READER:
			logfile = fopen(filename.c_str(), "r");
			break;
	}
	if(!logfile) {
		throw runtime_error("Konnte Logfile " + filename + " nicht öffnen");
	}
}

void VideoLogfile::operator<<(VideoFrame image) {
	if(logfile == 0) {
		throw runtime_error("Keine Logdatei geöffnet!");
	}
	if(mode != WRITER) {
		throw runtime_error("Kann nur Logfiles im WRITER Mode schreiben!");
	}
}

void VideoLogfile::operator>>(VideoFrame& outputFrame) {
	if(logfile == 0) {
		throw runtime_error("Keine Logdatei geöffnet!");
	}
	if(mode != READER) {
		throw runtime_error("Kann nur Logfiles im READER Mode lesen!");
	}
	int32_t width, height, frame_id;
	int64_t timestamp_ns;
	ArvPixelFormat format;

	fread(&width, sizeof(int32_t), 1, logfile);
	fread(&height, sizeof(int32_t), 1, logfile);
	fread(&frame_id, sizeof(int32_t), 1, logfile);
	fread(&timestamp_ns, sizeof(int64_t), 1, logfile);
	Mat image(Size(width, height), CV_8UC1);
	fread(image.data, width*height, 1, logfile);

	outputFrame.timestamp_ns = timestamp_ns;
	outputFrame.image = image;
	outputFrame.format = format;
	outputFrame.frame_id = frame_id;
}

bool VideoLogfile::eof() {
	if(logfile == 0) {
		throw runtime_error("Keine Logdatei geöffnet!");
	}
	return feof(logfile);
}
