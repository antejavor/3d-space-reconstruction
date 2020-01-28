#pragma once
#include <opencv2/core.hpp>


class Camera 
{

private:
	int id;
	cv::Mat intrinsic_matrix;
	cv::Mat distortion_coeffs;

	const std::string FILE_NAME;
	

public:

	Camera(int id) : id{ id } {};
	
	int get_id();
	void set_id(int id);
	cv::Mat get_distortion_coeffs();
	cv::Mat get_intrinsic_matrix();
	void calibrate(int sample_num, double delay, int board_width, int board_height);
	void save_properties_to_file(std::string file_name);

};


