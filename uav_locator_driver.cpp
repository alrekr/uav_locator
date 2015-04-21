#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "uav_locator.hpp"

using namespace std;
using namespace cv;

int main(void) {
	init_locate_uav();
	Mat src = imread("sample.png", CV_LOAD_IMAGE_GRAYSCALE);
	p3d p = locate_uav(src);
	cout << "\ncen_x:  " << p.cen_x << "\ncen_y:  " << p.cen_y << "\ntheta:  "
			<< p.theta << "\nDegree: " << rtod(p.theta) << endl;
	cout << "Program is done." << endl;
	return 0;
}
