#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#include <iostream>
#include <ctype.h>

using namespace std;
class stereo
{
public:
	enum DIRECTIONS{LEFT,RIGHT};
private:
	//Make some custom variables
	typedef std::vector<cv::Point2f> imagePointVector;
	typedef std::vector<cv::Point3f> wordlPointVector;


	//Intrinsic matrix and distoryion vector
	cv::Mat intrinsic_matrix[2];
	cv::Mat distortion_vector[2];

	//Stereo parameters
	cv::Mat R, T, E, F;
	cv::Mat R1, R2, P1, P2, Q;
	
	//Left and right images
	cv::Mat left_image;
	cv::Mat right_image;

	//Left and right points
	imagePointVector left_image_points;
	imagePointVector right_image_points;
public:
	//Get left and right intrinsic and distortion information
	//input - left_file, right_file - file names for the left and right xml files
	//(assuming that in each file they contain both the intrinsic and distortion information)
	int initialize_intrinsic_distortion(std::string left_file, std::string right_file);

	//Get stereo parameters
	int initialize_stereo_parameters(std::string stereo_file);
	
	//Get left or right images
	//Input:	im_in - image input
	//			dir	  - left or right
	void set_images(cv::Mat im_in, DIRECTIONS dir);

	//Get left or right feature points
	//Input:	img_pts -vector of image points
	//			dir	  - left or right
	void set_points(imagePointVector img_pts, DIRECTIONS dir);



	//This function performs triangulation on the left and right points
	//Input:	left_points, right_points - left and right points in R2
	//Output:	world_points - points in R3
	std::vector<cv::Point3f> triangulation();
	
	
	//Default initializer
	stereo();

	//Destructor
	~stereo();


};
