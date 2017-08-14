
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


#include "stereo.h"
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
	stereo stereo_class;
	std::string filename_left = "Left_Intrinsic_ovr.xml";
	std::string filename_right = "Right_Intrinsic_ovr.xml";
	std::string filename_stereo = "Stereo_Prop_ovr.xml";
	stereo_class.initialize_intrinsic_distortion(filename_left, filename_right);
	stereo_class.initialize_stereo_parameters(filename_stereo);

	

	while (1){
		
		cap >> input_image;
		
		
		

		std::vector<cv::KeyPoint> keypoints_output = h_circle.find_circles(input_image);
		cv::drawKeypoints(input_image, keypoints_output, input_image, cv::Scalar(200, 100, 200), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		
		std::vector<cv::Point2f> keyPoints_vector;
		cv::KeyPoint::convert(keypoints_output, keyPoints_vector);

		stereo_class.set_points(keyPoints_vector, stereo::DIRECTIONS::LEFT);
		stereo_class.set_points(keyPoints_vector, stereo::DIRECTIONS::RIGHT);

		auto output= stereo_class.triangulation();


		cv::imshow("keypoints", input_image);
		cv::waitKey(1);
	}








	return 0;
}


