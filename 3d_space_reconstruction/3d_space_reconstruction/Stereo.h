#pragma once
#include "Camera.h"
class Stereo
{
private:
	Camera camera_left, camera_right;
	cv::Mat R, T, E, F;
public:
	Stereo(Camera &cam_left, Camera &cam_right) : camera_left{ cam_left }, camera_right{ cam_right }{};
	void stereo_calibration_and_vision(int sample_num, double delay);
};

