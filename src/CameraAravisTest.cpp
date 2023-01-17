#include "CameraAravis.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <frame_helper/FrameHelper.h>

#include <semaphore.h>
#include <exception>

using namespace std;
using namespace camera;

/* Global Variables */
static bool finished = false;
static CameraAravis *cam;

/*
 * Signal Handler for Strg-C
 */
void atExit(int signal) {
	finished = true;
}

sem_t buffer_sync; 

/*
 * Callback that is called, when a new frame is received
 */
void newFrameCallback(const void* data) {
	cout << "New Frame Signal received!" << endl;
	sem_post(&buffer_sync);
}

int main(int argc, const char *argv[])
{
	//Register Handler for Ctrl-C Signal
	signal(SIGINT, atExit);
	sem_init(&buffer_sync, 0, 0);

	cam = new CameraAravis();
	cam->openCamera(argv[1]);
	cam->grab(camera::Stop);
	cam->setAttrib(camera::int_attrib::BinningX, 2);
	cam->setAttrib(camera::int_attrib::BinningY, 2);
	//cam->openCamera("Aravis-GV01");
	cam->setAttrib(camera::enum_attrib::WhitebalModeToAuto);
	cam->setAttrib(camera::double_attrib::FrameRate, 24);
	cam->setAttrib(camera::int_attrib::ExposureValue, 1000);
	cam->setAttrib(camera::int_attrib::GainValue, 40);
	// cam->setAttrib(camera::int_attrib::RegionX, 1000);
	// cam->setAttrib(camera::int_attrib::RegionY, 1000);
	cam->grab(camera::Continuously, 50);
	cam->setFrameSettings(base::samples::frame::frame_size_t(1248, 720) ,base::samples::frame::MODE_BAYER, 0, false);

	//cam->setCallbackFcn(newFrameCallback, 0);
	cv::namedWindow("Test");
	while(!finished) {
		base::samples::frame::Frame newFrame;
		if(cam->isFrameAvailable()) {
			if(cam->retrieveFrame(newFrame, 0)) {
				cout << "Mode: " << newFrame.getFrameMode() << " Init: " << newFrame.getStatus() << endl;
				cout << "Width:" << newFrame.getWidth() << " Height: " << newFrame.getHeight() << endl;
				// retrieveFrame returns true if we have got a new frame...
				cv::Mat myImage = frame_helper::FrameHelper::convertToCvMat(newFrame);
				cv::imshow("Test", myImage);

			} else {
				cout << "No Frame?!?!" << endl;
				//throw runtime_error("No Frame?!?");
			}
		} 
		cv::waitKey(1);
	}
	cam->grab(camera::Stop);

	//Destroy objects
	delete cam;

	return 0;
}
