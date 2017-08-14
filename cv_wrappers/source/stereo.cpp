#include "stereo.h"

stereo::stereo(){
	std::cout << "Initialize stereo class" << std::endl;

}


stereo::~stereo(){

}

int stereo::initialize_intrinsic_distortion(std::string left_file, std::string right_file){
	//Get left information
	cv::FileStorage f_output_left(left_file, cv::FileStorage::READ);
	if (!f_output_left.isOpened()){
		std::cout << "Error opening left info " << std::endl;
		return 0;
	}

	f_output_left["Intrinsic"] >> intrinsic_matrix[0];
	f_output_left["Distortion"] >> distortion_vector[0];

	f_output_left.release();


	//Get right information
	cv::FileStorage f_output_right(right_file, cv::FileStorage::READ);
	
	if (!f_output_right.isOpened()){
		std::cout << "Error opening right info " << std::endl;
		return 0;
	}

	f_output_right["Intrinsic"] >> intrinsic_matrix[1];
	f_output_right["Distortion"] >> distortion_vector[1];

	f_output_right.release();

	return 1;
}

int stereo::initialize_stereo_parameters(std::string stereo_file){
	cv::FileStorage f_output_stereo(stereo_file, cv::FileStorage::READ);
	
	
	if (!f_output_stereo.isOpened()){
		std::cout << "Error : Cannot open stereo parameters " << std::endl;
		return 0;
	}
	f_output_stereo["R"] >> R;
	f_output_stereo["T"] >> T;
	f_output_stereo["E"] >> E;
	f_output_stereo["F"] >> F;
	f_output_stereo["R1"] >> R1;
	f_output_stereo["R2"] >> R2;
	f_output_stereo["P1"] >> P1;
	f_output_stereo["P2"] >> P2;
	f_output_stereo["Q"] >> Q;
	f_output_stereo.release();
	return 1;
}

void stereo::set_images(cv::Mat im_in, DIRECTIONS dir){
	if (dir == DIRECTIONS::LEFT){
		left_image = im_in;
	}
	else if (dir == DIRECTIONS::RIGHT){
		right_image = im_in;
	}
	else {
		std::cout << "ERROR direction specified is nonexistent, choose left or right" << std::endl;
	}
}

void stereo::set_points(stereo::imagePointVector vec_in, DIRECTIONS dir){
	if (dir == DIRECTIONS::LEFT){
		left_image_points = vec_in;
	}
	else if (dir == DIRECTIONS::RIGHT){
		right_image_points = vec_in;
	}
	else {
		std::cout << "ERROR direction specified is nonexistent, choose left or right" << std::endl;
	}

}

std::vector<cv::Point3f> stereo::triangulation(){

	//Container to store final 3d pos
	stereo::wordlPointVector solution_container;

	int number_points;
	if ((left_image_points.size() == right_image_points.size())
		&& !right_image_points.empty()
		&& !left_image_points.empty()){
		number_points = left_image_points.size();
	}
	else {
		std::cout << "ERROR : ensure that left and right points are the same size . " << std::endl;
		return solution_container;

	}


	for (int counter = 0;counter < number_points; counter++){

		imagePointVector left_temp, right_temp;
		left_temp.push_back(left_image_points[counter]);
		right_temp.push_back(right_image_points[counter]);
		
		cv::Mat homogeneous_output;	
		cv::triangulatePoints(P1, P2, (left_temp), (right_temp), homogeneous_output);

		//Change from homogeneous to non homogeneous
		cv::Vec4f euclidean_output = homogeneous_output.col(0);

		euclidean_output[0] /= euclidean_output[3];
		euclidean_output[1] /= euclidean_output[3];
		euclidean_output[2] /= euclidean_output[3];
		
		
		solution_container.push_back(cv::Point3f(euclidean_output[0],euclidean_output[1],euclidean_output[2]));



		

	}

	return solution_container;

}