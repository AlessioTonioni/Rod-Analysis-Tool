/*
 * Rod.hpp
 *
 *  Created on: 21/feb/2015
 *      Author: alessio
 */
#include "Hole.hpp"

#ifndef INC_ROD_HPP_
#define INC_ROD_HPP_

using namespace cv;
using namespace std;

class Rods{
public:
	Point baricenter;
	Rect ROI;
	int area=0;
	float widthBaricenter=-1;
	RotatedRect enclosingRectangle;
	vector<Hole> holes;
	vector<Point> contours;
	bool approximated;
	int type(); //1 type A one holes, 2 type B two holes
	float widthAtBaricenter(Mat*labeledImage, Point* rightCorner, Point* leftCorner);
	float getAngle(bool precise);
	string getDescription();
	void applyTranslation(int x, int y);
	bool operator==(const Rods& r);
	bool operator!=(const Rods& r);
	float getDistanceFromMajorAxis(Point p);
	float getMajorAxisAngularcoefficient();
	void getVersorMajorAxis(float*versore);

private:
	void getExtreme(Point2f*topsx, Point2f*topdx);
	float getAngleFromMoments();
	float getangleFromRectangle();
};



#endif /* INC_ROD_HPP_ */
