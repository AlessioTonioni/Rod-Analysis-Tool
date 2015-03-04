/*
 * function.hpp
 *
 *  Created on: 21/feb/2015
 *      Author: alessio
 */
#include "Rod.hpp"


#ifndef INC_FUNCTION_HPP_
#define INC_FUNCTION_HPP_

enum distanceType{
	MANHATTAN,
	EUCLIDIAN
};

enum directionType{
	SCAN_X,
	SCAN_Y
};

#define PI 3.1415926535

using namespace cv;
using namespace std;

int getEulerNumber(Mat*source,int startx, int starty, int endx, int endy, int connectivity);
int getEulerNumber(Mat* source,Rect ROI,int connectivity );
vector<RotatedRect> findContoursAndBoxes(Mat thresholdImage, vector<vector<Point>>& contours,vector<Vec4i>& hierarchy);
float getDistance(Point a, Point b,distanceType tipo);
bool isInside(Point p, Rect ROI);
bool isInside(Point p, RotatedRect ROI);
Hole getHole(vector<Point> circonferenza);
vector<Rods> detachRods(Mat image, vector<Hole> holes, Rect ROI, bool isFast);
Mat detachBlob(Mat image);
Rods computeApproximatedRod(Mat image, RotatedRect boundingBox, vector<Hole> holes);
Point2f findFrontChange(Mat image, Point2f p, float angCoeff,int start, int end, directionType mode, int nChange);
void applyLabel(Mat*image,vector<Rods> rods);

#endif /* INC_FUNCTION_HPP_ */
