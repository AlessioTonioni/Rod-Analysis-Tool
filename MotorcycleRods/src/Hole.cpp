#include "inc/Hole.hpp"

Hole::Hole(Point p, float r){
	center=p;
	radius=r;
	spessore=-1;
}

bool Hole::isInside(Point p){
	if(spessore==-1)
		return (pow(p.x-center.x,2)+pow(p.y-center.y,2)-pow(radius,2))<=0;
	else
		return (pow(p.x-center.x,2)+pow(p.y-center.y,2)-pow(radius+spessore,2))<=0;
}

