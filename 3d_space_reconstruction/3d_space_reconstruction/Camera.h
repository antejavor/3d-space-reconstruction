#pragma once
#include <opencv2/core.hpp>


class Camera 
{
public:
	int id;
	cv::Size image_size;
	cv::Mat intrinsic_matrix;
	cv::Mat distortion_coeffs;
	cv::Mat map1, map2;
	Camera(int id) : id{ id } {};	

	void calibrate_form_video(int sample_num, double delay);
	void save_properties_to_file(std::string file_name_xml);
	void load_properties_from_file(std::string file_name_xml);
	void run_calibrated_stream();
	
	

};


