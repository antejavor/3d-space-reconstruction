#include "Camera.h"
#include "Stereo.h"


int main()
{
	Camera cam_l{ 2 };
	Camera cam_r{ 1 };
	cam_l.load_properties_from_file("cam_l.xml");
	cam_r.load_properties_from_file("cam_r.xml");
	Stereo st{ cam_l, cam_r };

	st.stereo_calibration(30, 1);
	st.save_properties_to_file("stereo.xml");
	st.load_properties_from_file("stereo.xml");
	st.stereo_SGBM();

}


