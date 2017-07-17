#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
using namespace cv;
using namespace std;
class optic_flow
{
private:
	//variables for optic flow
	cv::Mat gray, prevGray, image, frame, frame2;
	std::vector<cv::Point2f> points[2];
	vector<uchar> status;
	Point2f point;
public:
	
	//initialize key feature points to track.
	//Input: vector of points to track
	void initialize_points(std::vector<cv::Point2f>);

	//Run the LK optic flow algorithm, the input is the display window name
	void run_LK(std::string);

	//Some criterias
	TermCriteria termcrit;
	Size subPixWinSize;
	Size winSize;
	bool addRemovePt = false;

	//Get frame
	void get_frame(cv::Mat in_frame){ frame = in_frame; }

	//Boolean variable to determine if initial points are established
	bool initialized = false;

	//add points to optic flow
	void add_points(cv::Point2f in_point);

	//clear all points
	void clear_points(void);

	//returns tracked points
	std::vector<cv::Point2f> return_tracked_point(void){ return points[0]; }

	//returns the status of the tracked points
	std::vector<uchar> return_status(void){ return status; }

	//get status for tracked points
	void get_status(std::vector<uchar> in_status){ status = in_status; }

	//History points
	std::vector<cv::Point2f> history;

	optic_flow();
	~optic_flow();
};

