#include "inc/Rod.hpp"
#include "inc/function.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int Rods::type(){
	return holes.size();
}

float Rods::widthAtBaricenter(Mat*labeledImage, Point* rightCorner, Point* leftCorner){

	//compute baricenter width translating the shortest edge orientation along the baricenter
	float angle=getangleFromRectangle();
	if(angle<60 || angle>120 ){ //image rectangle is vertical
		float angCoeff=-1/getMajorAxisAngularcoefficient();
		*rightCorner=findFrontChange(*labeledImage,baricenter,angCoeff,baricenter.x+1,ROI.x+ROI.width,SCAN_X,1);
		*leftCorner=findFrontChange(*labeledImage,baricenter,angCoeff,baricenter.x-1, ROI.x,SCAN_X,1);
	} else {
		float angCoeff=-getMajorAxisAngularcoefficient();
		*rightCorner=findFrontChange(*labeledImage,baricenter,angCoeff,baricenter.y+1,ROI.y+ROI.height,SCAN_Y,1);
		*leftCorner=findFrontChange(*labeledImage,baricenter,angCoeff,baricenter.y-1,ROI.y,SCAN_Y,1);
	}
	widthBaricenter=getDistance(*rightCorner,*leftCorner,EUCLIDIAN);
	return widthBaricenter;
}

float Rods::getAngle(bool precise){
	if(precise){
		return getAngleFromMoments();
	} else {
		return getangleFromRectangle();
	}
}

float Rods::getangleFromRectangle(){
	if(enclosingRectangle.size.width < enclosingRectangle.size.height){
		return enclosingRectangle.angle+180;
	}else{
		return enclosingRectangle.angle+90;
	}
}

float Rods::getAngleFromMoments(){
	if(contours.size()!=0){
		Moments m=moments(contours,false);
		float angle=(-(atan((2*m.mu11)/(m.mu02-m.mu20))*180/PI)/2);
		return (angle<0)?-1*angle+90:angle;
	} else {
		return -1;
	}
}

string Rods::getDescription(){
	stringstream ss;
	ss<<"- Type: "<<((type()==1)?"A":"B")<<endl;
	ss<<"- Baricenter: ("<<baricenter.x<<","<<baricenter.y<<")"<<endl;
	ss<<"- Orientation: "<<getAngle(false)<<endl;
	float width=(enclosingRectangle.size.height<enclosingRectangle.size.width)?enclosingRectangle.size.height:enclosingRectangle.size.width;
	float length=(enclosingRectangle.size.height>enclosingRectangle.size.width)?enclosingRectangle.size.height:enclosingRectangle.size.width;
	ss<<"- Width: "<<width<<endl;
	ss<<"- Length: "<<length<<endl;
	ss<<"- Width at Baricenter: "<<widthBaricenter<<endl;
	ss<<"- N° of holes: "<<type()<<endl;
	for(size_t j=0; j<holes.size(); j++){
		ss<<"Holes N°"<<j<<": center("<<holes[j].center.x<<","<<holes[j].center.y
				<<") diameter: "<<holes[j].radius*2<<endl;
	}
	return ss.str();
}

void Rods::applyTranslation(int x, int y){
	baricenter.x+=x;
	baricenter.y+=y;
	enclosingRectangle.center.x+=x;
	enclosingRectangle.center.y+=y;
	for(size_t i=0; i<holes.size(); i++){
		holes[i].center.x+=x;
		holes[i].center.y+=y;
	}
}

bool Rods::operator==(const Rods& r){
	if(getDistance(baricenter,r.baricenter,EUCLIDIAN)<10)
		return true;
	else
		return false;
}

bool Rods::operator!=(const Rods& r){
	if(getDistance(baricenter,r.baricenter,EUCLIDIAN)<10)
		return false;
	else
		return true;
}

float Rods::getDistanceFromMajorAxis(Point p){
	float angCoeff=getMajorAxisAngularcoefficient();
	float q=-angCoeff*baricenter.x+baricenter.y;

	return abs(p.y-(angCoeff*p.x+q))/sqrt(1+angCoeff*angCoeff);

}

float Rods::getMajorAxisAngularcoefficient(){
	Point2f topsx;
	Point2f topdx;
	getExtreme(&topsx,&topdx);

	return (topsx.y-topdx.y)/(topsx.x-topdx.x);
}

void Rods::getVersorMajorAxis(float*versore){
	Point2f topsx;
	Point2f topdx;
	getExtreme(&topsx,&topdx);
	float norma=sqrt(pow(topdx.x-topsx.x,2)+pow(topdx.y-topsx.y,2));
	versore[0]=(topdx.x-topsx.x)/norma;
	versore[1]=(topdx.y-topsx.y)/norma;
}

void Rods::getExtreme(Point2f*topsx, Point2f*topdx){

	//find the major edge of the rotated enclosing rectangle
	Point2f corners[4];
	enclosingRectangle.points(corners);
	if(getDistance(corners[1],corners[0],EUCLIDIAN)>getDistance(corners[1],corners[2],EUCLIDIAN)){
		*topsx=corners[0];
		*topdx=corners[1];
	} else {
		*topsx=corners[1];
		*topdx=corners[2];
	}
}


