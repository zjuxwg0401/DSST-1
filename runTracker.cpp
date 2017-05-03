#include <opencv2/opencv.hpp>

#include <iostream>
#include "dsst_tracker.hpp"
#include "dsst_debug.hpp"


int main() {

	cf_tracking::DsstParameters paras;
	paras.enableTrackingLossDetection = true;
	paras.useFhogTranspose = true;
	paras.enableScaleEstimator = true;
	paras.psrThreshold = 0;
	paras.padding = 2.5;
	cf_tracking::DsstTracker tracker(paras);

	cv::Mat frame;
	long count;
	int64 tstart, tduration;
	int lostframe;

	//get init target box params from information file
	std::ifstream initInfoFile;
	initInfoFile.open("panda\\init.txt");
	std::string firstLine;
	std::getline(initInfoFile, firstLine);
	initInfoFile.close();
	float initX, initY, initWidth, initHegiht;
	char ch;
	std::istringstream ss(firstLine);
	ss >> initX, ss >> ch;
	ss >> initY, ss >> ch;
	ss >> initWidth, ss >> ch;
	ss >> initHegiht, ss >> ch;

	cv::Rect_<float> initRect = cv::Rect_<float>(initX, initY, initWidth - initX, initHegiht - initY);

	cv::VideoCapture capture("panda.mpg");
	if (!capture.isOpened()) 
		return -1;
	count = 0; 
	tstart = 0; 
	tduration = 0; 
	lostframe = 0;
	while (capture.read(frame)) {
		assert(!frame.empty());
		if (count == 0) {
			tstart = cv::getTickCount();
			tracker.reinit(frame, initRect);
			tduration = cv::getTickCount() - tstart;
			cv::rectangle(frame, initRect, cv::Scalar(0, 255, 0));
			cv::imshow("KCF", frame);
			cv::waitKey(0);
		}
		else {
			tstart = cv::getTickCount();
			bool _targetOnFrame = tracker.update(frame, initRect);
			tduration += cv::getTickCount() - tstart;
			if (!_targetOnFrame) {
				cv::Point_<double> tl = initRect.tl();
				cv::Point_<double> br = initRect.br();

				line(frame, tl, br, cv::Scalar(0, 0, 255), 4);
				line(frame, cv::Point_<double>(tl.x, br.y),
					cv::Point_<double>(br.x, tl.y), cv::Scalar(0, 0, 255), 4);
				++ lostframe;
			}
			else {
				cv::rectangle(frame, initRect, cv::Scalar(0, 255, 0));
			}
			cv::imshow("KCF", frame);
			if (cv::waitKey(1) == 27) break;
		}
		count++;

	}
	capture.release();
	std::cout << "Lost Frame:# " << lostframe << std::endl;
	std::cout << "FPS:# " << static_cast<double>(cv::getTickFrequency() * count / tduration) << std::endl;

	cv::destroyAllWindows;
	return 0;

}






	
