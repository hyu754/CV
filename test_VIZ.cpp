
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
#include "surf.h"
#include <opencv2/viz/vizcore.hpp>
#include "aruco_tools.h"
//#include <opencv2/aruco/charuco.hpp>

#include "viz_tools.h"

#include "optic_flow.h"
using namespace std;
using namespace cv;
using namespace cv::cuda;
GpuMat img1;
GpuMat img2;

int main(int argc, char* argv[])
{
	/*if (argc != 5)
	{
	help();
	return -1;
	}*/
		
#if 0
	GpuMat img1, img2;
	for (int i = 1; i < argc; ++i)
	{
		if (string(argv[i]) == "--left")
		{
			img1.upload(imread(argv[++i], IMREAD_GRAYSCALE));
			CV_Assert(!img1.empty());
		}
		else if (string(argv[i]) == "--right")
		{
			img2.upload(imread(argv[++i], IMREAD_GRAYSCALE));
			CV_Assert(!img2.empty());
		}
		else if (string(argv[i]) == "--help")
		{
			help();
			return -1;
		}
}
#endif // 0
	//Mat img_1 = Mat(1000, 1000, CV_8UC1);
	VideoCapture cap(0);
	if (!cap.isOpened()){
		return -1;
	}
	
	////Mat img_2 = Mat(1000, 1000, CV_8UC1);
	//Mat img_1 = imread("green_dots.PNG", IMREAD_COLOR);//object
	//	
	//Mat img_2 = imread("green_dots.PNG", IMREAD_COLOR);//scene
	//	
	//cv::resize(img_2, img_2, cv::Size(300, 300));
	//cv::resize(img_1, img_1, cv::Size(500, 500));
	//cv::imshow("original image", img_1);
	//cv::cvtColor(img_2, img_2, cv::COLOR_RGB2GRAY);
	//cv::cvtColor(img_1, img_1, cv::COLOR_RGB2GRAY);

	///*cv::Mat hsv_channels1[3];
	//cv::Mat hsv_channels2[3];
	//	
	//cv::split(img_1, hsv_channels1);
	//cv::split(img_2, hsv_channels2);

	//	
	//img_1 = hsv_channels1[2];
	//img_2 = hsv_channels2[2];*/

	//
	//cv::GaussianBlur(img_2, img_2, cv::Size(5, 5), 2);
	//imshow("1", img_1);
	//imshow("2", img_2);
	////cvtColor(img_1, img_1, CV_8UC1);
	//std::cout << "Initializing cuda image containers" << std::endl;
	////cvtColor(img_2, img_2, CV_8UC1);
	//imshow("1", img_1);
	//imshow("2", img_2);
	
	/// Create a window


	cv::Mat input_image;
	cap >> input_image;


	viz_tools viz_class;
	viz_class.set_window_size(input_image.size());
	viz_class.set_virtual_camera_intrinsics();
	
	viz_class.create_plane(cv::Vec2d(20, 20));
	viz_class.set_camera_position();
	viz_class.set_camera();
	stereo_ar ar_class;
	while (1){
		cap >> input_image;
		std::vector<cv::Point2f> out_vector_unordered = ar_class.find_aruco_center_four_corners(input_image);
		int text_counter = 0;
		for (auto ptr_auto = out_vector_unordered.begin(); ptr_auto != out_vector_unordered.end(); ++ptr_auto){
			cv::circle(input_image, *ptr_auto, 5, cv::Scalar(255, 100, 100), 5);
			cv::putText(input_image, std::to_string(text_counter), *ptr_auto, 1, 2, cv::Scalar(10, 200, 100));
			text_counter++;
		}
		cv::imshow("input_image", input_image);
		cv::waitKey(1);
		std::vector<int> indicies2d, indicies3d;
		indicies2d.push_back(0);
		indicies2d.push_back(1);
		indicies2d.push_back(2);
		indicies2d.push_back(3);

		indicies3d.push_back(0);
		indicies3d.push_back(1);
		indicies3d.push_back(3);
		indicies3d.push_back(2);

		//Temporary solution
		std::vector<cv::Point3f> points;
		

		cv::Vec2d size_plane = cv::Vec2d(20, 20);
		points.push_back(cv::Point3f(0, 0, 0));
		points.push_back(cv::Point3f(0, size_plane(1), 0));
		points.push_back(cv::Point3f(size_plane(0), 0, 0));
		points.push_back(cv::Point3f(size_plane(0), size_plane(1), 0));

		if (points.size() == out_vector_unordered.size()){
			cv:Affine3f pose= viz_class.solve_pnp_matrix(points, indicies3d, out_vector_unordered, indicies2d);
			cv::Mat augmented_image  = viz_class.augment_mesh(input_image,"plane", pose);
			cv::imshow("augmented image", augmented_image);
			cv::waitKey(1);

		}


		viz_class.render();

	}








	return 0;
}


