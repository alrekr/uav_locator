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
cv::Mat src, in;
std::vector<std::vector<cv::Point> > received;
double lowest = INT_MAX, match, theta;
std::vector<double> matches;
std::vector<std::vector<cv::Point> > original;
p3d _p_here;
bool set_up_complete = false;

p3d locate_uav(Mat _in) {
    if(set_up_complete == false) {
        exit(-1);
    }

    _p_here.cen_x = 0;
    _p_here.cen_y = 0;
    _p_here.theta = 4;
#ifdef DEBUG
    cout << "Loading input image" << endl;
	_in = imread("/home/alrekr/Pictures/UAS/frame_194.png", CV_LOAD_IMAGE_GRAYSCALE);
	cout << "Input image loaded" << endl;
#endif //DEBUG
    received = get_shapes(_in);
#ifdef DEBUG
	cout << "Shapes for input image received" << endl;
	cout << "Contours in input image: " << received.size() << endl;
#endif //DEBUG
    for(i = 0; i < (int)received.size(); i++) {
#ifdef DEBUG
		cout << "Run " << i << " in matching shapes" << endl;
#endif //DEBUG
/* FIXME: There's a snake in my boot! */
        matches.push_back(matchShapes(original[ORIGINAL_SHAPE], received[i], CV_CONTOURS_MATCH_I1, 0));
#ifdef DEBUG
		cout << "Match " << i << " completed" << endl;
#endif //DEBUG
    }
#ifdef DEBUG
	cout << "Found matches, finding lowest number" << endl;
#endif //DEBUG
    for(i = 0; i < (int)matches.size(); i++) {
        match = matches[i];
#ifdef DEBUG
        cout << "Match is " << match << endl;
#endif //DEBUG
        if(lowest > match && match < MATCH_SHAPE_THRESHOLD) {
            lowest = match;
            best_match = i;
        }
    }

    if (lowest != INT_MAX) {
        get_orientation(received, best_match, _p_here);
        get_center(received, best_match, _p_here);
#ifdef DEBUG
        cout << "Angle is " << rtod(_p_here.theta) << endl;
#endif //DEBUG
    }
#ifdef DEBUG
    else {
        cout << "Best match was " << lowest << endl;
    }
#endif //DEBUG
#ifdef DEBUG
	cout << "Program is done." << endl;
#endif //DEBUG
	return _p_here;
}

/*****************************************************************************
 * Initialises some global variables
 * Input: Mat
 * Output: None
 *****************************************************************************/
void init_locate_uav(void) {
    Mat src = imread(SAMPLE_IMAGE, CV_LOAD_IMAGE_GRAYSCALE);
	if(!src.data) {
#ifdef DEBUG
		cout << "Original image not loaded!!" << endl;
#endif //DEBUG
		exit(-2);
	}
#ifdef DEBUG
	else {
		cout << "Original image loaded." << endl;
	}
#endif //DEBUG
    original = get_shapes(src);
#ifdef DEBUG
	cout << "Shapes in original image identified." << endl;
	cout << "Contours in original image: " << original.size() << endl;
#endif //DEBUG
    set_up_complete = true;
}

/*****************************************************************************
 * Prepares a Mat for thresholding.
 * Input: address to a Mat
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
 * Input: Address to a Mat
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
 * Input: Mat
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
 * Calculates the orientation of the found shape, based on the found contours.
 * Function is based on
 * http://stackoverflow.com/questions/14720722/binary-image-orientation
 * Input: Found contours and which contour matches with the original sample.
 * Output: Angle in radians
 *****************************************************************************/
void get_orientation(vector<vector<Point> > _contours, int _which_one, p3d &_p) {
	//based on
	double _theta;
	Moments _m = moments(_contours[_which_one], false);
	_p.theta = 0.5 * atan2(((-2) * _m.mu11), (_m.mu20 - _m.mu02));
}

/*****************************************************************************
 * Calculates the center of the contour
 * Input: vector<Point>, address of resulting p3d
 * Output: None
 *****************************************************************************/
void get_center(vector<vector<Point> > _contour, int _n, p3d &_p) {
    Moments _m = moments(_contour[_n]);
    _p.cen_x= _m.m10/_m.m00;
    _p.cen_y = _m.m01/_m.m00;
}

/*****************************************************************************
 * Converts radians to degree
 * Input: double radian to convert
 * Output: double degree
 *****************************************************************************/
double rtod(double _r) {
    return _r*180/M_PIl;
}

/*****************************************************************************
 * Converts degree to radians
 * Input: double degree to convert
 * Output: double radians
 *****************************************************************************/
double dtor(double _d) {
    return _d*M_PIl/180;
}
