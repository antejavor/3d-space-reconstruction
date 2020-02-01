#include "Stereo.h"
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

void Stereo::stereo_calibration_and_vision(int sample_num, double delay, int board_width, int board_height)
{
	std::cout << "Start of calibration function!\n";

	int      board_num = board_width * board_height;
	cv::Size board_size = cv::Size(board_width, board_height);
	cv::VideoCapture capture_left(camera_left.id);

	if (!capture_left.isOpened())
	{
		std::cerr << "Couldn't open the camera\n";
		return;
	}

	std::vector<std::vector<cv::Point2f> > image_points_left;
	std::vector<std::vector<cv::Point3f> > object_points;
	cv::Size image_size_left;
	double   last_captured_timestamp_l = 0;
	cv::namedWindow("Calibration", cv::WINDOW_FREERATIO);
	while (image_points_left.size() < (size_t)sample_num)
	{

		cv::Mat image;
		capture_left >> image;
		image_size_left = image.size();

		std::vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_size, corners);
		drawChessboardCorners(image, board_size, corners, found);

		double timestamp = (double)clock() / CLOCKS_PER_SEC;
		if (found && timestamp - last_captured_timestamp_l > delay)
		{

			last_captured_timestamp_l = timestamp;

			image_points_left.push_back(corners);
			object_points.push_back(std::vector<cv::Point3f>());
			std::vector<cv::Point3f>& opts = object_points.back();
			opts.resize(board_num);
			for (int j = 0; j < board_num; j++) {
				opts[j] = cv::Point3f((float)(j / board_width), (float)(j % board_width), 0.f);
			}
			std::cout << "Collected our " << (int)image_points_left.size() <<
				" of " << sample_num << " needed chessboard images\n" << std::endl;
		}
		cv::imshow("Calibration", image);
		if ((cv::waitKey(30) & 255) == 27)
			return;
	}
	capture_left.release();
	cv::destroyAllWindows();
	std::vector<std::vector<cv::Point2f> > image_points_right;

	cv::VideoCapture capture_right(camera_right.id);
	if (!capture_right.isOpened())
	{
		std::cerr << "Couldn't open the camera\n";
		return;
	}

	double   last_captured_timestamp = 0;
	cv::namedWindow("Calibration", cv::WINDOW_FREERATIO);
	while (image_points_right.size() < (size_t)(sample_num))
	{

		cv::Mat image;
		capture_right >> image;


		std::vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_size, corners);
		drawChessboardCorners(image, board_size, corners, found);

		double timestamp = (double)clock() / CLOCKS_PER_SEC;
		if (found && timestamp - last_captured_timestamp > delay)
		{

			last_captured_timestamp = timestamp;

			image_points_right.push_back(corners);
			object_points.push_back(std::vector<cv::Point3f>());
			std::vector<cv::Point3f>& opts = object_points.back();
			opts.resize(board_num);
			for (int j = 0; j < board_num; j++) {
				opts[j] = cv::Point3f((float)(j / board_width), (float)(j % board_width), 0.f);
			}
			std::cout << "Collected our " << (int)image_points_right.size() <<
				" of " << sample_num << " needed chessboard images\n" << std::endl;
		}
		cv::imshow("Calibration", image);
		if ((cv::waitKey(30) & 255) == 27)
			return;
	}
	capture_left.release();
	cv::destroyAllWindows();

	std::vector< std::vector<cv::Point3f> >::iterator row;
	std::vector<cv::Point3f>::iterator col;

	for (row = object_points.begin(); row != object_points.end(); row++) {
		for (col = row->begin(); col != row->end(); col++) {
			std::cout << *col << '\n';
		}
	}
	
	cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat D1, D2, R, T, E, F;
	std::cout << "\nRunning stereo calibration ...\n";
	cv::stereoCalibrate(
		object_points,
		image_points_left,
		image_points_right,
		M1, D1, M2, D2,
		image_size_left, R, T, E, F,
		cv::CALIB_FIX_ASPECT_RATIO,
		cv::TermCriteria(
			cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5
		)

	);
	std::cout << "Done\n\n";



	std::cout << "\n\n*** CALIBRATING THE CAMERA...\n" << std::endl;

	return;

}