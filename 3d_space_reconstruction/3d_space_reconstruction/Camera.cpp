#include "Camera.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

int Camera::get_id() 
{
	return id;
}

void Camera::set_id(int ) 
{
	this->id = id;
}

cv::Mat Camera::get_distortion_coeffs() 
{
	return distortion_coeffs;
}

cv::Mat Camera::get_intrinsic_matrix() 
{
	return intrinsic_matrix;
}

void Camera::calibrate(int sample_num, double delay, int board_width, int board_height)
{
	std::cout << "Start of calibration function!\n";
	int      board_num = board_width * board_height;
	cv::Size board_size = cv::Size(board_width, board_height);
	cv::VideoCapture capture(id);
	if (!capture.isOpened())
	{
		std::cerr << "Couldn't open the camera\n";
		return;
	}

	std::vector<std::vector<cv::Point2f> > image_points;
	std::vector<std::vector<cv::Point3f> > object_points;

	double   last_captured_timestamp = 0;
	cv::Size image_size;
	cv::namedWindow("Calibration", cv::WINDOW_FREERATIO);
	while (image_points.size() < (size_t)sample_num)
	{

		cv::UMat image;
		capture >> image;

		std::vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_size, corners);
		drawChessboardCorners(image, board_size, corners, found);

		double timestamp = (double)clock() / CLOCKS_PER_SEC;
		if (found && timestamp - last_captured_timestamp > delay)
		{
			last_captured_timestamp = timestamp;


			image_points.push_back(corners);
			object_points.push_back(std::vector<cv::Point3f>());
			std::vector<cv::Point3f>& opts = object_points.back();
			std::cout << "Collected our " << (int)image_points.size() <<
				" of " << board_num << " needed chessboard images\n" << std::endl;
		}
		cv::imshow("Calibration", image);  //show in color if we did collect the image

		if ((cv::waitKey(30) & 255) == 27)
			return;
	}
	cv::destroyWindow("Calibration");
	std::cout << "\n\n*** CALIBRATING THE CAMERA...\n" << std::endl;

	// CALIBRATE THE CAMERA!
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
	);
}

void Camera::save_properties_to_file(std::string file_name)
{


}