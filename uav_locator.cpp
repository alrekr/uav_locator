/*
 * TODO: Make into ROS node.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "ros/time.h"
#include "uav_locator.hpp"

using namespace cv;
using namespace std;

int i, best_match;
Mat in;
vector<vector<Point> > received;
double lowest = INT_MAX, match, theta;
vector<double> matches;
vector<vector<Point> > original;
Point3d _p_here;
bool set_up_complete = false;

/*****************************************************************************
 * Main entry point for the program.
 * Input:  Mat, which might contain a UAV
 * Output: Object of type Point3d with information about the found UAV
 *****************************************************************************/
Point3d locate_uav(Mat _in) {
    if(set_up_complete == false) {
        exit(-1);
    }

    _p_here.x = 0;
    _p_here.y = 0;
    _p_here.z = 4;
    received = get_shapes(_in);

    for(i = 0; i < (int)received.size(); i++) {
        matches.push_back(matchShapes(original[ORIGINAL_SHAPE], received[i], CV_CONTOURS_MATCH_I1, 0));
    }

    for(i = 0; i < (int)matches.size(); i++) {
        match = matches[i];

        if(lowest > match && match < MATCH_SHAPE_THRESHOLD) {
            lowest = match;
            best_match = i;
        }
    }

    if (lowest != INT_MAX) {
        get_orientation(received, best_match, _p_here);
    }

	return _p_here;
}

/*****************************************************************************
 * Initialises some global variables
 * Input:  Mat
 * Output: None
 *****************************************************************************/
void init_locate_uav(void) {
    Mat src = imread(SAMPLE_IMAGE, CV_LOAD_IMAGE_GRAYSCALE);

	if(!src.data) {
        cout << "'sample.png' not found. Exiting." << endl;
		exit(-2);
	}

    original = get_shapes(src);
    set_up_complete = true;
}

/*****************************************************************************
 * Prepares a Mat for thresholding.
 * Input:  address to a Mat
 * Output: none
 *****************************************************************************/
void prepare_mat(Mat &_src) {
	Mat _element;
	blur(_src, _src, Size(5,5));
	_element = getStructuringElement(MORPH_ELLIPSE,
			Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE+1),
			Point(EROSION_SIZE, EROSION_SIZE));
	erode(_src, _src, _element, Point(-1, -1), ERODE_PREP_ITERATIONS);
}

/*****************************************************************************
 * Performs erosion and dilation on a Mat, to make individual contours.
 * Input:  Address to a Mat
 * Output: None
 *****************************************************************************/
void erode_dilate(Mat &_src) {
    Mat _element = getStructuringElement(MORPH_ELLIPSE,
			Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE+1),
			Point(EROSION_SIZE, EROSION_SIZE));
	erode(_src, _src, _element, Point(-1,-1), ERODE_ITERATIONS);
	dilate(_src, _src, _element, Point(-1,-1), DILATE_ITERATIONS);
}

/*****************************************************************************
 * Calls prepare_mat(), threshold(), erode_dilate(), and findContours() in
 * order to find shapes in the image.
 * Input:  Mat
 * Output: cds (custom struct for storing contour and hierarchy)
 *****************************************************************************/
vector<vector<Point> > get_shapes(Mat _src) {
	vector<vector<Point> > _contours;
	vector<Vec4i> _hierarchy;

	prepare_mat(_src);
	threshold(_src, _src, THRESHOLD, BINARY_MAX, THRESHOLD_MODE);
	erode_dilate(_src);
	findContours(_src, _contours, _hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	return _contours;
}

/*****************************************************************************
 * Calculates the orientation and mass center of the found shape, based on the
 * found contours. Function is based on
 * http://stackoverflow.com/questions/14720722/binary-image-orientation
 * Input:  Found contours
 *         Which contour matches with the original sample
 *         Object of type Point3d to save information to
 * Output: None
 *****************************************************************************/
void get_orientation(vector<vector<Point> > _contours, int _n, Point3d &_p) {
	Moments _m = moments(_contours[_n], false);
    _p.x = _m.m10/_m.m00;
    _p.y = _m.m01/_m.m00;
	_p.z = 0.5 * atan2(((-2) * _m.mu11), (_m.mu20 - _m.mu02));

}

/*****************************************************************************
 * Converts radians to degree
 * Input:  double radian to convert
 * Output: double degree
 *****************************************************************************/
double rtod(double _r) {
    return _r*180/M_PIl;
}

/*****************************************************************************
 * Converts degree to radians
 * Input:  double degree to convert
 * Output: double radians
 *****************************************************************************/
double dtor(double _d) {
    return _d*M_PIl/180;
}
