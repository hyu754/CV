	#if 1
	#include <iostream>

	#include "opencv2/opencv_modules.hpp"
	#define HAVE_OPENCV_XFEATURES2D true
	#ifdef HAVE_OPENCV_XFEATURES2D

	#include "opencv2/core.hpp"
	#include "opencv2/features2d.hpp"
	#include "opencv2/highgui.hpp"
	#include "opencv2/cudafeatures2d.hpp"
	#include "opencv2/xfeatures2d/cuda.hpp"

	#include <opencv2/cudastereo.hpp>
	#include <opencv2/cudaimgproc.hpp>
	//#include <opencv2/aruco/charuco.hpp>

	using namespace std;
	using namespace cv;
	using namespace cv::cuda;
	GpuMat img1;
	GpuMat img2;
	static void help()
	{
		cout << "\nThis program demonstrates using SURF_CUDA features detector, descriptor extractor and BruteForceMatcher_CUDA" << endl;
		cout << "\nUsage:\n\tmatcher_simple_gpu --left <image1> --right <image2>" << endl;
	}

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
		//Mat img_2 = Mat(1000, 1000, CV_8UC1);
		Mat img_1 = imread("green_dots.PNG", IMREAD_COLOR);//object
		
		Mat img_2 = imread("green_dots.PNG", IMREAD_COLOR);//scene
		
		cv::resize(img_2, img_2, cv::Size(300, 300));
		cv::resize(img_1, img_1, cv::Size(500, 500));
		cv::imshow("original image", img_1);
		cv::cvtColor(img_2, img_2, cv::COLOR_RGB2GRAY);
		cv::cvtColor(img_1, img_1, cv::COLOR_RGB2GRAY);

		/*cv::Mat hsv_channels1[3];
		cv::Mat hsv_channels2[3];
		
		cv::split(img_1, hsv_channels1);
		cv::split(img_2, hsv_channels2);

		
		img_1 = hsv_channels1[2];
		img_2 = hsv_channels2[2];*/

		cv::GaussianBlur(img_1, img_1, cv::Size(5, 5), 2);
		cv::GaussianBlur(img_2, img_2, cv::Size(5, 5), 2);
		imshow("1", img_1);
		imshow("2", img_2);
		//cvtColor(img_1, img_1, CV_8UC1);
		std::cout << "Initializing cuda image containers" << std::endl;
		//cvtColor(img_2, img_2, CV_8UC1);
		imshow("1", img_1);
		imshow("2", img_2);

		img1.create(1, 1, CV_8U);
		img2.create(1, 1, CV_8U);
		img1.upload(img_1);//object

		img2.upload(img_2);//scene
		//Mat outputimg2 = img1.download();
		//imshow("a",Mat(img2));
		cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

		SURF_CUDA surf(300);


		// detecting keypoints & computing descriptors

		Mat Frame;
		cap >> Frame;
		cv::cvtColor(Frame, Frame, CV_BGR2GRAY);
		img1.upload(Frame);


		for (;;){
			cap >> Frame;
			cv::cvtColor(Frame, Frame, CV_BGR2GRAY);
			/*cv::Mat channels[3];
			cv::split(Frame, channels);
			Frame = channels[2];*/
			cv::GaussianBlur(Frame, Frame, cv::Size(5, 5), 2);
			cv::imshow("camera", Frame);
			//for (int i = 0; i < 20; i++){
			img1.upload(img_1);
			img2.upload(Frame);//scene
			GpuMat keypoints1GPU, keypoints2GPU;
			GpuMat descriptors1GPU, descriptors2GPU;
			surf(img1, GpuMat(), keypoints1GPU, descriptors1GPU);
			surf(img2, GpuMat(), keypoints2GPU, descriptors2GPU);

			cout << "FOUND " << keypoints1GPU.cols << " keypoints on first image" << endl;
			cout << "FOUND " << keypoints2GPU.cols << " keypoints on second image" << endl;

			// matching descriptors
			Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(surf.defaultNorm());
			vector<DMatch> matches;
			//matcher->knnMatch(descriptors1GPU, descriptors2GPU, matches);
			if (keypoints2GPU.cols != 0){
				matcher->match(descriptors1GPU, descriptors2GPU, matches);

				// downloading results
				vector<KeyPoint> keypoints1, keypoints2;
				vector<float> descriptors1, descriptors2;

				surf.downloadKeypoints(keypoints1GPU, keypoints1);
				surf.downloadKeypoints(keypoints2GPU, keypoints2);
				surf.downloadDescriptors(descriptors1GPU, descriptors1);
				surf.downloadDescriptors(descriptors2GPU, descriptors2);

				// drawing the results
				Mat img_matches;
				double max_dist = 0; double min_dist = 1000;
				for (int k = 0; k < matches.size(); k++)
				{
					double dist = matches[k].distance;
					if (dist < min_dist) min_dist = dist;
					if (dist > max_dist) max_dist = dist;
				}

				std::vector< DMatch > good_matches;

				for (int l = 0; l < matches.size(); l++)
				{
					if (matches[l].distance < 2.2 * min_dist)
					{
						good_matches.push_back(matches[l]);
					}
				}
				drawMatches(Mat(img1), keypoints1, Mat(img2), keypoints2, good_matches, img_matches,cv::Scalar(0,20,100));

				//namedWindow("matches", 0);
				imshow("matches", img_matches);
				
			}
			waitKey(1);
		}

		//}

		waitKey(0);

		return 0;
	}

	#else

	int main()
	{
		std::cerr << "OpenCV was built without xfeatures2d module" << std::endl;
		return 0;
	}

	#endif  
	#endif // 0
