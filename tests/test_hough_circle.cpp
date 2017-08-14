
#include <iostream>
#include <ctime>
#include "opencv2/opencv_modules.hpp"



#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"

#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "hough_circle.h"
using namespace std;
using namespace cv;
using namespace cv::cuda;


int main(int argc, char* argv[])
{


	VideoCapture cap(0);
	if (!cap.isOpened()){
		return -1;
	}

	cv::Mat input_image;
	cap >> input_image;

	hough_circle h_circle;
	while (1){
		cap >> input_image;
		
		std::vector<cv::KeyPoint> keypoints_output = h_circle.find_circles(input_image);
		cv::drawKeypoints(input_image, keypoints_output, input_image, cv::Scalar(200, 100, 200), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		
		cv::imshow("keypoints", input_image);
		cv::waitKey(1);
	}








	return 0;
}


