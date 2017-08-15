
#include <iostream>
#include <map>
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
#include "optic_flow.h"
#include <ovrvision_pro.h>
using namespace std;
using namespace cv;
using namespace cv::cuda;


int main(int argc, char* argv[])
{


	//VideoCapture cap(0);
	//if (!cap.isOpened()){
	//	return -1;
	//}

	//cv::Mat input_image;
	//cap >> input_image;

	//Hough circle class
	hough_circle h_circle;

	//Stereo class
	stereo stereo_class;


	//hough circle ball tracking class
	hough_circle_ball_tracker tracker_class[2];



	//Getting camera properties
	std::string filename_left = "Left_Intrinsic_ovr.xml";
	std::string filename_right = "Right_Intrinsic_ovr.xml";
	std::string filename_stereo = "Stereo_Prop_ovr.xml";
	stereo_class.initialize_intrinsic_distortion(filename_left, filename_right);
	stereo_class.initialize_stereo_parameters(filename_stereo);

	/*
	Set up ovrvision
	*/
	OVR::OvrvisionPro ovrvision;

	OVR::Camprop cameraMode = OVR::OV_CAMHD_FULL;
	if (ovrvision.Open(0, cameraMode) == 0){
		std::cout << "OVRVISION camera not initialized " << std::endl;
		return -1;
	}
	
	/*
	Cam height and width
	*/
	int ovr_height, ovr_width;
	ovr_height = ovrvision.GetCamHeight();
	ovr_width = ovrvision.GetCamWidth();
	//Image containers for the ovrvision
	cv::Mat im_left = cv::Mat(ovr_height, ovr_width, CV_8UC4);
	cv::Mat im_right = cv::Mat(ovr_height, ovr_width, CV_8UC4);
	

	//Set the exposure, this is a picked value
	ovrvision.SetCameraExposure(5400);

	//Simple mapper that will map the left oright
	std::map<int, int> left_right_mapper;
	while (1){
		
	/*	cap >> input_image;*/
		std::vector<cv::KeyPoint> keypoints_output_left;
		std::vector<cv::KeyPoint> keypoints_output_right;

		//lk image, these images will only have a dot at the centre of circle
		cv::Mat lk_image_left, lk_image_right;
		if (ovrvision.isOpen()){
			ovrvision.PreStoreCamData(OVR::Camqt::OV_CAMQT_DMS);
			unsigned char* p_image = ovrvision.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT);
			unsigned char* p2_image = ovrvision.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT);

			im_left.data = p_image;
			im_right.data = p2_image;
			if (stereo_class.roi_is_set() == false){
				stereo_class.set_roi(im_left, stereo::DIRECTIONS::LEFT);
				stereo_class.set_roi(im_right, stereo::DIRECTIONS::RIGHT);
			}
			else{
				keypoints_output_left= h_circle.find_circles(im_left, stereo_class.return_roi(stereo::LEFT), 1.0, "left");
				keypoints_output_right= h_circle.find_circles(im_right, stereo_class.return_roi(stereo::RIGHT),1.0, "right");
			}
			



			if (!keypoints_output_left.empty() && !keypoints_output_right.empty()){
				cv::Mat left_gray, right_gray;
				cv::cvtColor(im_left, left_gray, CV_RGBA2GRAY);
				cv::cvtColor(im_right, right_gray, CV_RGBA2GRAY);
				cv::drawKeypoints(left_gray, keypoints_output_left, left_gray, cv::Scalar(200, 100, 200), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				cv::drawKeypoints(right_gray, keypoints_output_right, right_gray, cv::Scalar(200, 100, 200), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				char key_press1 = cv::waitKey(1);

				//LEFT tracker
				if ((tracker_class[0].return_initialized_flag() == false) && (key_press1 == ' ' )){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_left, keypointvector);
					tracker_class[0].initialize_points(keypointvector);
					tracker_class[0].set_initialized_flag(true);
				}
				else if(tracker_class[0].return_initialized_flag() == true){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_left, keypointvector);
					//tracker_class.initialize_points(keypointvector);
					tracker_class[0].get_current_points(keypointvector);
					tracker_class[0].run(10.0);
					tracker_class[0].draw_image(im_left,"left");
				}

				//right tracker
				if ((tracker_class[1].return_initialized_flag() == false) && (key_press1 == ' ')){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_right, keypointvector);
					tracker_class[1].initialize_points(keypointvector);
					tracker_class[1].set_initialized_flag(true);
				}
				else if (tracker_class[1].return_initialized_flag() == true){
					std::vector<cv::Point2f> keypointvector;
					cv::KeyPoint::convert(keypoints_output_right, keypointvector);
					//tracker_class.initialize_points(keypointvector);
					tracker_class[1].get_current_points(keypointvector);
					tracker_class[1].run(10.0);
					tracker_class[1].draw_image(im_right,"right");
				}

				/*std::vector<cv::Point2f> keyPoints_vector_left, keyPoints_vector_right;
				cv::KeyPoint::convert(keypoints_output_left, keyPoints_vector_right);

				stereo_class.set_points(keyPoints_vector_left, stereo::DIRECTIONS::LEFT);
				stereo_class.set_points(keyPoints_vector_right, stereo::DIRECTIONS::RIGHT);

				auto output = stereo_class.triangulation();*/

	
				
			
				


				cv::imshow("left points", left_gray);
				cv::imshow("right points ", right_gray);
			}

			cv::imshow("keypoints left", im_left);
			cv::imshow("keypoints right", im_right);
			char keypress = cv::waitKey(1);

			if (keypress == '='){
				ovrvision.SetCameraExposure(ovrvision.GetCameraExposure()+50);
			}else if (keypress == '-') {
				ovrvision.SetCameraExposure(ovrvision.GetCameraExposure() - 50);
			}
			std::cout << "Exposure : " << ovrvision.GetCameraExposure() << std::endl;


			if (keypress == 'c'){
				std::vector<int> tracer_status_left = tracker_class[0].return_status();
				std::vector<int> tracer_status_right = tracker_class[1].return_status();
				int counter = 0;
				//for (auto left_status_ptr = tracer_status_left.begin(); left_status_ptr != tracer_status_left.end(); ++left_status_ptr){
				//	
				//	if (*left_status_ptr == 1){
				//		std::cout << "Which point on the left : " << std::to_string(counter) << " correspond to what point of right? : ";
				//		int right_point;
				//		std::cin >> right_point;
				//		if (right_point != -1) // if -1 we skip this point
				//			left_right_mapper[counter] = right_point;
				//	}
				//	counter++;

				//}
				//populate mapper

				int left_pair=0, right_pair=0;
				std::cout << "Please enter pairs, to stop pairs enter -1 " << std::endl;
				while (left_pair != -1){
					
					std::cout << std::to_string(counter) << " st/th pair " << std::endl;
					std::cout << "Please enter left index : ";
					std::cin >> left_pair;
					if (left_pair == -1) break;
					std::cout << "Please enter right index: ";
					std::cin >> right_pair;
					left_right_mapper[left_pair] = right_pair;
					
				}


			}
		}
	}








	return 0;
}


