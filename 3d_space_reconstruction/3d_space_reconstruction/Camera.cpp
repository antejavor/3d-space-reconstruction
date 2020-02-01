#include "Camera.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void Camera::calibrate_form_video(int sample_num, double delay)
{
	std::cout << "Start of calibration function!\n";
	int board_width = 9;
	int board_height = 6;
	int      corners_num = board_width * board_height;
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
	cv::namedWindow("Calibration", cv::WINDOW_FREERATIO);
	while (image_points.size() < (size_t)sample_num)
	{

		cv::Mat image;
		capture >> image;
		image_size = image.size();
		
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
			opts.resize(corners_num);
			for (int j = 0; j <corners_num; j++) {
				opts[j] = cv::Point3f((float)(j / board_width), (float)(j % board_width), 0.f);
			}
			std::cout << "Collected our " << (int)image_points.size() <<
				" of " << sample_num << " needed chessboard images\n" << std::endl;
		}
		cv::imshow("Calibration", image);
		if ((cv::waitKey(30) & 255) == 27)
			return;
	}
	capture.release();
	cv::destroyAllWindows();
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

	std::cout << " *** DONE! " << "Reprojection error is " << err << "\n";
	std::cout << " *** DONE! " << "Intrinsic matrix is " << intrinsic_matrix << "\n";
	std::cout << " *** DONE! " << "Distorsion coefficient is " << distortion_coeffs << "\n";

	std::cout << "Build undistortion map for video stream\n";

	cv::initUndistortRectifyMap(
		intrinsic_matrix,
		distortion_coeffs,
		cv::Mat(),
		intrinsic_matrix,
		image_size,
		CV_16SC2,
		map1,
		map2
	);

}

void Camera::save_properties_to_file(std::string file_name_xml)
{

	std::cout << "Storing properties to file " << file_name_xml << "\n";
	cv::FileStorage fs(file_name_xml, cv::FileStorage::WRITE);

	fs << "image_width" << image_size.width
		<< "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix
		<< "distortion_coefficients" << distortion_coeffs
		<< "map1" << map1
		<< "map2" << map2;
	fs.release();
	std::cout << "***DONE! Storing properties to file" << file_name_xml << "\n";
}

void Camera::load_properties_from_file(std::string file_name_xml)
{
	std::cout << "Load properties from " << file_name_xml << "\n";
	cv::FileStorage fs(file_name_xml, cv::FileStorage::READ);
	fs["image_width"] >> image_size.width;
	fs["image_height"] >> image_size.height;
	fs["camera_matrix"] >> intrinsic_matrix;
	fs["distortion_coefficients"] >> distortion_coeffs;
	fs["map1"] >> map1;
	fs["map2"] >> map2;
	std::cout << "\nimage width: " << image_size.width;
	std::cout << "\nimage height: " << image_size.height;
	std::cout << "\nintrinsic matrix:" << intrinsic_matrix;
	std::cout << "\ndistortion coefficients: " << distortion_coeffs;
	std::cout << "***DONE! Loading properties from file" << file_name_xml << std::endl;
	fs.release();
}

void Camera::run_calibrated_stream() {

	std::cout << "Start stream\n";
	cv::namedWindow("Calibrated stream", cv::WINDOW_FREERATIO);
	cv::VideoCapture capture(id);
	if (!capture.isOpened())
	{
		std::cerr << "Couldn't open the camera\n";
		return;
	}

	for (;;) {
		cv::UMat image, image0;
		capture >> image0;
		if (image0.empty() || map1.empty()) break;
		cv::remap(
			image0,
			image,
			map1,
			map2,
			cv::INTER_LINEAR,
			cv::BORDER_CONSTANT,
			cv::Scalar()
		);
		cv::imshow("Calibrated stream", image);
		if ((cv::waitKey(30) & 255) == 27) break;
	}
	capture.release();
	cv::destroyAllWindows();
	std::cout << "End stream\n";
}



