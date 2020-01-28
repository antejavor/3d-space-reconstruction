#include "Camera.h"

void Camera::start_callibration_process(int sample_num, double delay, int board_width, int board_height)
{
	std::cout << "Start of calibration function!\n";
	int      board_num = board_width * board_height;
	cv::Size board_size = cv::Size(board_width, board_height);

	cv::VideoCapture capture(camera_id);
	if (!capture.isOpened())
	{
		std::cerr << "Couldn't open the camera\n";
		return;
	}

	std::vector<std::vector<cv::Point2f> > image_points;
	std::vector<std::vector<cv::Point3f> > object_points;

	double   last_captured_timestamp = 0;
	cv::Size image_size;

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
}

void Camera::save_camera_properties()
{

}