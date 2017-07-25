#pragma once

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

#include "aruco_tools.h"

#include <opencv2/viz/vizcore.hpp>




class viz_tools
{
private:
	//Viz camera pointer
	cv::viz::Viz3d *myWindow;

	//Intrinsic matrix for the camera
	cv::Mat camMat;

	//window width and height
	cv::Size size_window;

	//Camera of the viz3d window
	cv::viz::Camera *camera;

	//Camera position matrix
	cv::Affine3f cam_pose;

	//Transformation to make view direction the same
	Affine3f transform;// = viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, -1.0f), cam_pos);

public:

	//Get geometry

	//Create a plane of size
	void create_plane(cv::Vec2d size_plane);

	//Set window size
	void set_window_size(cv::Size winSize);

	//Set camera matrix, call after window size has been set
	void set_virtual_camera_intrinsics(void);

	//Set the camera of viz3d window to be user defined camera
	//Note the render must have rendered one scene before this function
	//can be called
	void set_camera(void);

	//Set the camera position with the same focal lengths, etc, from the camera being used
	void set_camera_position(void);

	//Render the current scene to the window
	void render(void){ myWindow->spinOnce(1,false); }

	//Wrapper function to solve for the PnP-problem
	//Inputs: points_3d - 3d points, points_2d - 2d points
	//Outputs: Affine3f matrix (or extrinsic matrix)
	
	Affine3f solve_pnp_matrix(std::vector<cv::Point3f>, std::vector<cv::Point2f>); //Ordered case 
	Affine3f solve_pnp_matrix(std::vector<cv::Point3f>, std::vector<int> indices_3d, std::vector<cv::Point2f>, std::vector<int> indices_2d); //Unordered case

	//Augment mesh function will transform the virtual model onto the camera fram
	//String_name will specify the name of the geometry,
	//Pose, is the new pose of the virtual object from solve_pnp_matrix
	cv::Mat augment_mesh(cv::Mat input_image,std::string object_name,cv::Affine3f pose);

	//constructors
	viz_tools();
	viz_tools(std::string window_name);
	~viz_tools();
};

