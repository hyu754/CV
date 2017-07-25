#include "viz_tools.h"


viz_tools::viz_tools(){
	myWindow = new cv::viz::Viz3d("default window");
}

viz_tools::viz_tools(std::string window_name){
	myWindow = new cv::viz::Viz3d(window_name);
	
}

void viz_tools::create_plane(cv::Vec2d size_plane){

	cv::viz::Mesh plane;
	std::vector<Vec3d> points;
	std::vector<int> polygons;

	points.push_back(Vec3d(0, 0, 0));
	points.push_back(Vec3d(0, size_plane(1), 0));
	points.push_back(Vec3d(size_plane(0), 0, 0));
	points.push_back(Vec3d(size_plane(0), size_plane(1), 0));


	polygons.push_back(4);
	polygons.push_back(3);
	polygons.push_back(2);
	polygons.push_back(1);
	polygons.push_back(0);



	plane.cloud = Mat(points, true).reshape(3, 1);
	plane.polygons = Mat(polygons, true).reshape(1, 1);

	plane.colors = cv::viz::Color::blue();
	
	myWindow->showWidget("plane", cv::viz::WMesh(plane));
	myWindow->setRenderingProperty("plane", cv::viz::LINE_WIDTH, 4);

	std:vector<cv::viz::WLine > line_vector;
	line_vector.push_back(cv::viz::WLine(points.at(3), points.at(2), cv::viz::Color::orange()));
	line_vector.push_back(cv::viz::WLine(points.at(2), points.at(1), cv::viz::Color::orange()));
	line_vector.push_back(cv::viz::WLine(points.at(1), points.at(0), cv::viz::Color::orange()));
	line_vector.push_back(cv::viz::WLine(points.at(0), points.at(3), cv::viz::Color::orange()));



	int _line_pos = 0;
	/*for (auto line_ptr = line_vector.begin(); line_ptr != line_vector.end(); ++line_ptr){
		line_ptr->setRenderingProperty(cv::viz::LINE_WIDTH, 4);
		myWindow->showWidget(std::to_string(_line_pos), cv::viz::WLine(*line_ptr));
		_line_pos++;
	}*/
	

}



void viz_tools::set_window_size(cv::Size winSize){
	myWindow->setWindowSize(winSize);
	size_window = winSize;
}

void viz_tools::set_virtual_camera_intrinsics(void){
	//focal length
	float f = cv::max(size_window.width,size_window.height);

	float cx = size_window.width / 2.0f;
	float cy = size_window.height / 2.0f;

	 
	camMat = (Mat_<float>(3, 3) <<f, 0, cx,0, f, cy,0, 0, 1);

}

void viz_tools::set_camera(void){

	float fx = camMat.at<float>(0, 0);
	float fy = camMat.at<float>(1, 1);
	float cx = camMat.at<float>(0, 2);
	float cy = camMat.at<float>(1, 2);
	//initialize the camera with predetermined camera matrix
	camera = new cv::viz::Camera(fx, fy, cx, cy, size_window);

	//set the window camera to be the camera above
	//Note: the renderer must have rendered one scene
	myWindow->spinOnce();
	myWindow->setCamera(*camera);

}

void viz_tools::set_camera_position(void){

	float f = cv::max(size_window.width, size_window.height);
	float cx = size_window.width / 2.0f;
	float cy = size_window.height / 2.0f;

	
	Vec3f cam_pos(cx, cy, f);
	Vec3f cam_focal_point(cx, cy, 0);
	Vec3f cam_y_dir(-1.0, 0.0, 0.0f);
	cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

	transform= viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, -1.0f), cam_pos);
	myWindow->setViewerPose(cam_pose);
}


Affine3f viz_tools::solve_pnp_matrix(std::vector<cv::Point3f>_p3d /*3d points*/, std::vector<cv::Point2f>/*3d points*/_p2d){
	cv::Mat rout = Mat::zeros(1, 3, CV_32F);
	cv::Mat tout = Mat::zeros(1, 3, CV_32F);

	cv::solvePnP( _p2d, _p3d,camMat, cv::Mat(), rout, tout);

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


	Affine3f pose(rot_mat, cv::Vec3f(tout_temp.at(0), tout_temp.at(1), tout_temp.at(2)));

	return pose;

}


Affine3f viz_tools::solve_pnp_matrix(
	std::vector<cv::Point3f>_p3d_unordered /*3d points*/,
	std::vector<int> _i3 /*3d indicies*/,
	std::vector<cv::Point2f>/*3d points*/_p2d_unordered,
	std::vector<int> _i2 /*2d indicies*/)
{
	cv::Mat rout = Mat::zeros(1, 3, CV_32F);
	cv::Mat tout = Mat::zeros(1, 3, CV_32F);


	std::vector<cv::Point3f> _p3d;
	std::vector<cv::Point2f> _p2d;

	//order the unordered vectors 3d
	for (auto _i3_ptr = _i3.begin(); _i3_ptr != _i3.end(); ++_i3_ptr){
		_p3d.push_back(_p3d_unordered.at(*_i3_ptr));
	}

	//order the unordered vectors 2d
	for (auto _i2_ptr = _i2.begin(); _i2_ptr != _i2.end(); ++_i2_ptr){
		_p2d.push_back(_p2d_unordered.at(*_i2_ptr));
	}

	cv::solvePnP( _p3d, _p2d,camMat, cv::Mat(), rout, tout);

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


	Affine3f pose(rot_mat, cv::Vec3f(tout_temp.at(0), tout_temp.at(1), tout_temp.at(2)));

	return pose;

}

cv::Mat viz_tools::augment_mesh(cv::Mat input_image,std::string object_name, cv::Affine3f pose){
	pose = transform*pose;
	//Affine3f cloud_pose_global = transform * cloud_pose;
	myWindow->setWidgetPose(object_name, pose);

	cv::Mat screenshot = myWindow->getScreenshot();
	cv::Mat dst;
	cv::addWeighted(input_image, 1, screenshot, 1, 0, dst);

	return dst;
}

viz_tools::~viz_tools(){
	//std::cout << "SURF WRAPPER DESTROYED" << std::endl;
}
