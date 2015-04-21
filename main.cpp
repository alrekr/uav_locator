#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "uav_locator.hpp"

using namespace std;
using namespace cv;

int main(void) {
	//init_locate_uav();
	Point3d p;
	for(int cnt = 0; cnt < 1000; cnt++) {
		Mat src = imread("test.png", CV_LOAD_IMAGE_GRAYSCALE);
		p = locate_uav(src);
	}
	/*cout << "cen_x:  " << p.x << "\ncen_y:  " << p.y << "\ntheta:  "
			<< p.z << "\nDegree: " << rtod(p.z) << endl;*/
	cout << "Program is done." << endl;
	return 0;
}
