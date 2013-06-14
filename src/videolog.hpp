#include <string>
#include <cv.h>
#include <cstdio>
#include <arv.h>

enum VideoLogMode {
	WRITER, READER
};

struct VideoFrame {
	int64_t timestamp_ns;
	cv::Mat image;
	ArvPixelFormat format;
	int32_t frame_id;
};

class VideoLogfile {
	public:
		void open(std::string filename, VideoLogMode mode);
		void operator<<(VideoFrame image);
		void operator>>(VideoFrame& image);
		bool eof();
	private:
		FILE* logfile;
		VideoLogMode mode;
};
