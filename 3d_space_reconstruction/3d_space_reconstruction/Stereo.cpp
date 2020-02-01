#include "Stereo.h"
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

void Stereo::stereo_calibration_and_vision(int sample_num, double delay)
{
	std::cout << "Start of calibration function!\n";
	int board_width = 9;
	int board_height = 6;
	int      board_num = board_width * board_height;
	cv::Size board_size = cv::Size(board_width, board_height);
	cv::VideoCapture capture_left(camera_left.id);
	cv::VideoCapture capture_right(camera_right.id);

	if (!capture_left.isOpened() && !capture_right.isOpened())
	{
		std::cerr << "Couldn't open the camera\n";
		return;
	}	

	std::vector<std::vector<cv::Point2f> > image_points_right;
	std::vector<std::vector<cv::Point2f> > image_points_left;
	std::vector<std::vector<cv::Point3f> > object_points;
	cv::Size image_size_left;
	double   last_captured_timestamp_l = 0;
	cv::namedWindow("Calibration_left", cv::WINDOW_FREERATIO);
	cv::namedWindow("Calibration_right", cv::WINDOW_FREERATIO);

	while (image_points_left.size() < (size_t)sample_num)
	{

		cv::Mat image_l, image_r;
		capture_left >> image_l;
		capture_right >> image_r;

		image_size_left = image_l.size();
		std::vector<cv::Point2f> corners_l;
		std::vector<cv::Point2f> corners_r;

		bool found_l = cv::findChessboardCorners(image_l, board_size, corners_l);
		drawChessboardCorners(image_l, board_size, corners_l, found_l);

		bool found_r = cv::findChessboardCorners(image_r, board_size, corners_r);
		drawChessboardCorners(image_r, board_size, corners_r, found_r);

		double timestamp = (double)clock() / CLOCKS_PER_SEC;
		if (found_l && found_r && timestamp - last_captured_timestamp_l > delay)
		{
			last_captured_timestamp_l = timestamp;

			image_points_left.push_back(corners_l);
			image_points_right.push_back(corners_r);
			object_points.push_back(std::vector<cv::Point3f>());
			std::vector<cv::Point3f>& opts = object_points.back();
			opts.resize(board_num);
			for (int j = 0; j < board_num; j++) {
				opts[j] = cv::Point3f((float)(j / board_width), (float)(j % board_width), 0.f);
			}

			std::cout << "Collected our left: " << (int)image_points_left.size()
				<< "and right:" << (int)image_points_right.size() <<
				" of " << sample_num << " needed chessboard images\n" << std::endl;
		}
		cv::imshow("Calibration_left", image_l);
		cv::imshow("Calibration_right", image_r);
		if ((cv::waitKey(30) & 255) == 27)
			return;
	}
	capture_left.release();
	capture_right.release();
	cv::destroyAllWindows();
	
	std::cout << "\nRunning stereo calibration ...\n";
	cv::stereoCalibrate(
		object_points,
		image_points_left,
		image_points_right,
		camera_left.intrinsic_matrix,
		camera_left.distortion_coeffs,
		camera_right.intrinsic_matrix,
		camera_right.distortion_coeffs,
		image_size_left, R, T, E, F,
		cv::CALIB_USE_INTRINSIC_GUESS,
		cv::TermCriteria(
			cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5
		)

	);
	std::cout << "Done\n\n";


	return;

}