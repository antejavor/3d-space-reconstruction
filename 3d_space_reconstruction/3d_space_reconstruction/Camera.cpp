#include "Camera.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int Camera::get_id()
{
	return id;
}

void Camera::set_id(int)
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

cv::Size Camera::get_image_size()
{
	return image_size;
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
			opts.resize(board_num);
			for (int j = 0; j < board_num; j++) {
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

	std::cout << "\nStoring properties to file " << file_name_xml << "\n";
	cv::FileStorage fs(file_name_xml, cv::FileStorage::WRITE);

	fs << "image_width" << image_size.width << "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
		<< distortion_coeffs;
	fs.release();
}

void Camera::load_properties_from_file(std::string file_name_xml)
{
	cv::FileStorage fs(file_name_xml, cv::FileStorage::READ);
	std::cout << "\nimage width: " << (int)fs["image_width"];
	std::cout << "\nimage height: " << (int)fs["image_height"];

	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	std::cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
	std::cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << std::endl;
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