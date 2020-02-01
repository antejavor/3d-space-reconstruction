#pragma once
#include "Camera.h"
class Stereo
{
private:
	Camera camera_left, camera_right;
	cv::Mat R, T, E, F;
	cv::Mat rectification_l, rectification_r;
	cv::Mat projection_l, projection_r;
	cv::Mat map_l1, map_l2, map_r1, map_r2;
public:
	Stereo(Camera &cam_left, Camera &cam_right) : camera_left{ cam_left }, camera_right{ cam_right }{};
	void stereo_calibration_and_vision(int sample_num, double delay);
};

