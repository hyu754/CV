
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
	cv::viz::Viz3d myWindow("Viz Demo");
	/// Add coordinate axes
	//myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

	/// Start event loop
	//myWindow.spin();

	/*viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1, 1, 1));
	axis.setRenderingProperty(viz::LINE_WIDTH, 4);
	myWindow.showWidget("Line Widget", axis);*/


	viz::WCube cube_widget(Point3f(0.5, 0.5, 0.0), Point3f(0.0, 0.0, -0.5), true, viz::Color::blue());

	cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
	/// Display widget (update if already displayed)
	//myWindow.showWidget("Cube Widget", cube_widget);

	/// Rodrigues vector
	Mat rot_vec = Mat::zeros(1, 3, CV_32F);
	float translation_phase = 0.0, translation = 0.0;
	myWindow.setBackgroundColor(cv::viz::Color(0, 0, 0));

	
	std::vector<Vec3d> points,point2;
	std::vector<Vec2d> tcoords,tcoords2;
	std::vector<int> polygons,polygons2;
	for (size_t i = 0; i < 64; ++i)
	{
		double angle = CV_PI / 2 * i / 64.0;
		points.push_back(Vec3d(0.00, cos(angle), sin(angle))*0.75);
		points.push_back(Vec3d(1.57, cos(angle), sin(angle))*0.75);
		tcoords.push_back(Vec2d(0.0, i / 64.0));
		tcoords.push_back(Vec2d(1.0, i / 64.0));
	}


	cv::Mat input_image;
	cap >> input_image;
	int rows = input_image.rows;
	int cols = input_image.cols;

	float aruco_length =2000;
	point2.push_back(Vec3d(-aruco_length / 2, -aruco_length / 2, 10));
	point2.push_back(Vec3d(-aruco_length / 2, aruco_length / 2, 10));
	point2.push_back(Vec3d(aruco_length / 2, -aruco_length / 2, 10));
	point2.push_back(Vec3d(aruco_length / 2, aruco_length / 2, 10));

	tcoords2.push_back(Vec2d(1, 1));
	tcoords2.push_back(Vec2d(1, 0));
	tcoords2.push_back(Vec2d(0, 1));
	tcoords2.push_back(Vec2d(0, 0));
	
	
	cv::viz::WImage3D image_3d(input_image,input_image.size());

	


	for (int i = 0; i < (int)points.size() / 2 - 1; ++i)
	{
		int polys[] = { 3, 2 * i, 2 * i + 1, 2 * i + 2, 3, 2 * i + 1, 2 * i + 2, 2 * i + 3 };
		polygons.insert(polygons.end(), polys, polys + sizeof(polys) / sizeof(polys[0]));
	}
	int polys2[] = { 0, 1, 2, 3 };
	polygons2.push_back(4);
	polygons2.push_back(3);
	polygons2.push_back(2);
	polygons2.push_back(1);
	polygons2.push_back(0);

	cv::viz::Mesh mesh;
	cv::viz::Mesh plane;
	cv::viz::Mesh geo=	cv::viz::readMesh("parasaurolophus_6700.ply");
//	geo.load("parasaurolophus_6700.obj", cv::viz::Mesh::LOAD_OBJ);

	mesh.cloud = Mat(points, true).reshape(3, 1);
	mesh.tcoords = Mat(tcoords, true).reshape(2, 1);
	mesh.polygons = Mat(polygons, true).reshape(1, 1);

	plane.cloud = Mat(point2, true).reshape(3, 1);
	plane.tcoords = Mat(tcoords2, true).reshape(2, 1);
	plane.polygons = Mat(polygons2, true).reshape(1, 1);


	mesh.texture = input_image;
	
	//myWindow.showWidget("mesh", cv::viz::WMesh(mesh));
	myWindow.showWidget("plane", cv::viz::WMesh(plane));
	//myWindow.showWidget("dinosaur", cv::viz::WMesh(geo));

	
	myWindow.setWindowSize(cv::Size(input_image.cols, input_image.rows));
	/// Let's assume camera has the following properties
	float focal_length =- 640.0f;




	int _counter_ = 0;
	Vec3f cam_pos(0, 0, focal_length ), cam_focal_point(0, 0, 100), cam_y_dir(1.0, 0.0, 0.0f);
	Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	

	
	//myWindow.showWidget("image", image_3d);



	stereo_ar ar_class;
//	float _counter_ = 0;

	//get camera matrix
	float f = 640.0f; 
	cv::Mat camMat = (Mat_<float>(3, 3) << f, 0, input_image.cols / 2,
		0, f, input_image.rows / 2,
		0, 0, 1);

	myWindow.showWidget("plane", cv::viz::WMesh(plane));


	/*double fx = camMat.at<float>(0, 0);
	double fy = camMat.at<float>(1, 1);
	double cx = camMat.at<float>(0, 2);
	double cy = camMat.at<float>(1, 2);
	cv::Mat persp;
	.create(4, 4)
	cam_pose(0, 0) = fx / cx;
	cam_pose(1, 1) = fy / cy;
	cam_pose(2, 2) = -(far + near) / (far - near);
	persp(2, 3) = -2.0*far*near / (far - near);
	persp(3, 2) = -1.0;*/
	//double fx = camMat.at<float>(0, 0);
	//double fy = camMat.at<float>(1, 1);
	//double cx = camMat.at<float>(0, 2);
	//double cy = camMat.at<float>(1, 2);
	//cam_pose.matrix.val[0] = fx / cx;
	//cam_pose.matrix.val[1] = 0.0f;
	//cam_pose.matrix.val[2] = 0.0f;
	//cam_pose.matrix.val[3] = 0.0f;
	//double near =0.1, far = 100;
	//cam_pose.matrix.val[4] = 0.0f;
	//cam_pose.matrix.val[5] = fy / cy;
	//cam_pose.matrix.val[6] = 0.0f;
	//cam_pose.matrix.val[7] = 0.0f;

	//cam_pose.matrix.val[8] = 0.0f;
	//cam_pose.matrix.val[9] = 0.0f;
	//cam_pose.matrix.val[10] = -(far + near) / (far - near);
	//cam_pose.matrix.val[11] = -2.0*far*near / (far - near);

	//cam_pose.matrix.val[12] = 0.0f;
	//cam_pose.matrix.val[13] = 0.0f;
	//cam_pose.matrix.val[14] = -1.0f;
	//cam_pose.matrix.val[15] = 0.0f;

	//
	//Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, -1.0f), cam_pos);
	//
	//Matx44d temp_matrix = cam_pose.matrix;
	////cv::viz::Camera camera(temp_matrix, cv::Size(input_image.cols, input_image.rows));
	//cv::viz::Camera camera(fx, fy, cx, cy, cv::Size(input_image.cols, input_image.rows));
	myWindow.setViewerPose(cam_pose);
	
	float _count_ = 0;
	while (!myWindow.wasStopped())
	{
		//myWindow.setViewerPose(cam_pose);
		//camera = myWindow.getCamera();
	//	myWindow.setCamera(camera);
		float time_start = std::clock();
		
		cap >> input_image;
		std::vector<cv::Point2f> out_vector_unordered = ar_class.find_aruco_center_four_corners(input_image);
		std::vector<cv::Point2f> out_vector;
		if (out_vector_unordered.size() == 4){

			out_vector.push_back(out_vector_unordered.at(1));
			out_vector.push_back(out_vector_unordered.at(2));
			out_vector.push_back(out_vector_unordered.at(3));
			out_vector.push_back(out_vector_unordered.at(0));
		}
	//	myWindow.setViewerPose(cam_pose);


		/*point2.clear();
		point2.push_back(Vec3d(-200 / 2, -200 / 2, _counter_));

		point2.push_back(Vec3d(-200 / 2, 200 / 2, _counter_));
		point2.push_back(Vec3d(200 / 2, -200 / 2, _counter_));
		point2.push_back(Vec3d(200 / 2, 200 / 2, _counter_));
		plane.cloud = Mat(point2, true).reshape(3, 1);*/
		//myWindow.showWidget("plane", cv::viz::WMesh(plane));
		
		cv::Mat screenshot= myWindow.getScreenshot();

		Vec3f cam_pos(0, 0,0), cam_focal_point(0, 0, 100), cam_y_dir(1.0, 0.0, 0.0f);
		_count_++;
		Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
		//myWindow.setViewerPose(cam_pose);
		cv::Mat dst;

		cv::addWeighted(input_image, 1, screenshot, 1, 0, dst);
		cv::imshow("screen shot", screenshot);
		int text_counter = 0;
		for (auto ptr_auto = out_vector.begin(); ptr_auto != out_vector.end(); ++ptr_auto){
			cv::circle(dst, *ptr_auto, 5, cv::Scalar(255, 100, 100),5);
			cv::putText(dst, std::to_string(text_counter), *ptr_auto,1,2,cv::Scalar(10,200,100));
			text_counter++;
		}
		cv::Mat rout = Mat::zeros(1, 3, CV_32F);
		cv::Mat tout= Mat::zeros(1, 3, CV_32F);
		cv::Vec3f tout_3f;
		if (point2.size() == out_vector.size()){
			std::vector<Vec3d> orderedpoint2;
			orderedpoint2.push_back(point2.at(0));
			orderedpoint2.push_back(point2.at(1));
			orderedpoint2.push_back(point2.at(3));
			orderedpoint2.push_back(point2.at(2));
			cv::solvePnP(orderedpoint2, out_vector, camMat, cv::Mat(), rout, tout);
			
			
		
	
			/// Construct pose
			
			std::vector < float> rout_temp;
			rout.copyTo(rout_temp);
			Mat rot_vec = Mat::zeros(1, 3, CV_32F);
			rot_vec.at<float>(0, 0) = rout_temp.at(0);
			rot_vec.at<float>(0, 1) = rout_temp.at(1);
			rot_vec.at<float>(0, 2) = rout_temp.at(2);

			std::vector<float> tout_temp;
			tout.copyTo(tout_temp);

			
			//std::cout << temp_vector << std::endl;
			//tout.copyTo(rot_vec)
			Mat rot_mat = Mat::Mat(3, 3, CV_32F);
			Rodrigues(rot_vec, rot_mat);

			
			Affine3f pose(rot_mat, cv::Vec3f(tout_temp.at(0),tout_temp.at(1), tout_temp.at(2)));

			//pose = cam_pose*pose;

			myWindow.setWidgetPose("plane", pose);
			//myWindow.setViewerPose(cam_pose);
			//myWindow.showWidget("plane", cv::viz::WMesh(plane));
			
		}

		cv::imshow("morphed", dst);
		//myWindow.setBackgroundTexture(input_image);
		///* Rotation using rodrigues */
		///// Rotate around (1,1,1)
		//rot_vec.at<float>(0, 0) += CV_PI * 0.01f;
		//rot_vec.at<float>(0, 1) += CV_PI * 0.01f;
		//rot_vec.at<float>(0, 2) += CV_PI * 0.01f;


		////myWindow.showWidget("plane", cv::viz::WMesh(plane));
		//translation_phase += CV_PI * 0.01f;
		//translation = sin(translation_phase);

	/*	Mat rot_mat;
		Rodrigues(rot_vec, rot_mat);

		/// Construct pose
		Affine3f pose(rot_mat, Vec3f(translation, translation, translation));

		myWindow.setWidgetPose("Cube Widget", pose);*/
	
		myWindow.spinOnce(1, false);

		std::cout << "FPS cv : " << 1.0/((time_start - std::clock()) / CLOCKS_PER_SEC) << std::endl;
	}

	return 0;
}


