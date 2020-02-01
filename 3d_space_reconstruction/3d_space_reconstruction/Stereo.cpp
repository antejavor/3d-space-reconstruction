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
	std::cout << "\nRunning stereo rectifiy ...\n";

	stereoRectify(
		camera_left.intrinsic_matrix,
		camera_left.distortion_coeffs,
		camera_left.intrinsic_matrix,
		camera_left.distortion_coeffs,
		image_size_left,
		R, T, rectification_l, rectification_r, projection_l, projection_r,
		cv::noArray(), cv::CALIB_ZERO_DISPARITY
	);

	std::cout << "***DONE!\n";

	std::cout << "\nRunning undistort maps ...\n";
	cv::initUndistortRectifyMap(
		camera_left.intrinsic_matrix,
		camera_left.distortion_coeffs,
		rectification_l, projection_l, image_size_left, CV_16SC2, map_l1, map_l2
	);
	cv::initUndistortRectifyMap(
		camera_right.intrinsic_matrix,
		camera_right.distortion_coeffs,
		rectification_r,
		projection_l, image_size_left, CV_16SC2, map_r1, map_r2
	);
	std::cout << "***DONE!\n";

	cv::Mat pair;
	pair.create(image_size_left.height, image_size_left.width * 2, CV_8UC3);
	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
		-64, 128, 11, 100, 1000,
		32, 0, 15, 1000, 16,
		cv::StereoSGBM::MODE_HH);

	while (true) {
		cv::UMat image_l, image_r;
		capture_left >> image_l;
		capture_right >> image_r;

		cv::UMat imgl, imgr, disp, vdisp;
		cv::remap(image_l, imgl, map_l1, map_l2, cv::INTER_LINEAR);
		cv::remap(image_r, imgr, map_r1, map_r2, cv::INTER_LINEAR);
		
		stereo->compute(imgl, imgr, disp);
		cv::normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
		cv::imshow("disparity", vdisp);

		cv::Mat part = pair.colRange(0, image_size_left.width);
		cvtColor(imgl, part, cv::COLOR_BGR2GRAY);
		part = pair.colRange(image_size_left.width, image_size_left.width * 2);
		cvtColor(imgr, part, cv::COLOR_BGR2GRAY);

		for (int j = 0; j < image_size_left.height; j += 16)
			cv::line(
				pair,
				cv::Point(0, j),
				cv::Point(image_size_left.width * 2, j),
				cv::Scalar(0, 255, 0)
			);

		cv::imshow("rectified", pair);
		if ((cv::waitKey(30) & 255) == 27)
			return;
	}

	capture_left.release();
	capture_right.release();
	cv::destroyAllWindows();

	// Setup for finding stereo corrrespondences
	 //


	return;

}