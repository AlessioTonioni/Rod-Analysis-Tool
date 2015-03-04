/*
 * Hole.hpp
 *
 *  Created on: 21/feb/2015
 *      Author: alessio
 */
#include <opencv2/core/core.hpp>
#ifndef INC_HOLE_HPP_
#define INC_HOLE_HPP_

using namespace cv;

class Hole{
public:
	Point center;
	float radius;
	float spessore;
	Hole(Point p, float r);
	bool isInside(Point p);
};


#endif /* INC_HOLE_HPP_ */
