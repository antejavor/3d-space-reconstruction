#pragma once
#include <opencv2/core.hpp>


class Camera {

private:
	const std::string FILE_NAME;
	void save_camera_properties();

public:
	Camera(int id, std::string file_name) : camera_id{ id }, FILE_NAME{ file_name }  {};
	int camera_id;
	cv::Mat intrinsic_matrix;
	cv::Mat distortion_coeffs;
	void start_callibration_process(int sample_num, double delay, int board_width, int board_height);

};

