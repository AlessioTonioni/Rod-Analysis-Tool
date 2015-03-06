#include "inc/function.hpp"
#include "inc/Hole.hpp"
#include "inc/Rod.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/utility.hpp>
#include <iostream>
#include <functional>

//Implementation
int getEulerNumber(Mat*source,int startx, int starty, int endx, int endy, int connectivity=4){
	//source image must be binarized: 255 for foreground object 0 for background.
	if(connectivity!=4 && connectivity!=8){
		return NAN;
	}
	int countQ1=0;
	int countQ3=0;
	int countQD=0;
	for(int i=startx; i<endx; i++){
		for(int j=starty; j<endy; j++){
			int OOValue=source->data[j*source->step+i];
			int OIValue=source->data[j*source->step+i+1];
			int IOValue=source->data[(j+1)*source->step+i];
			int IIValue=source->data[(j+1)*source->step+i+1];
			switch(OOValue+OIValue+IOValue+IIValue){
			case 255:
				countQ1++;
				break;
			case 765:
				countQ3++;
				break;
			case 510:
				if(OOValue==IIValue)
					countQD++;
				break;
			}
		}
	}
	if(connectivity==4)
		return (countQ1-countQ3+2*countQD)/4;
	else
		return (countQ1-countQ3-2*countQD)/4;
}

int getEulerNumber(Mat* source,Rect ROI,int connectivity=4 ){
	return getEulerNumber(source,(int)ROI.x,(int)ROI.y,(int)(ROI.x+ROI.width),(int)(ROI.y+ROI.height),connectivity);
}

vector<RotatedRect> findContoursAndBoxes(Mat thresholdImage, vector<vector<Point>>& contours,vector<Vec4i>& hierarchy ){
	// Find contours
	findContours( thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Find the rotated rectangles for each contour
	vector<RotatedRect> minRect(contours.size());
	for( size_t i = 0; i < contours.size(); i++ )
		minRect[i] = minAreaRect(Mat(contours[i]));

	return minRect;
}

float getDistance(Point a, Point b,distanceType tipo){
	switch(tipo){
	case EUCLIDIAN:
		return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
	case MANHATTAN:
		return abs(a.x-b.x)+abs(a.y-b.y);
	}
	return 0;
}

bool isInside(Point p, Rect ROI){
	return p.x>=ROI.x && p.y>=ROI.y && p.x<=(ROI.x+ROI.width) && p.y<=(ROI.y+ROI.height);
}

bool isInside(Point p, RotatedRect ROI){
	/*a rotated rect is described by two vector or three corners
	describing a reference system aligned with the edges of the rectangle.
	In order to be inside the rect the vector conecting the point with the origin
	of the reference system must lay inside this two vectors*/
	Point2f vertex[4];
	ROI.points(vertex);
	Point2f yVertex=vertex[0];
	Point2f origin=vertex[1];
	Point2f xVertex=vertex[2];

	Mat yAxis(1,2,DataType<float>::type);
	yAxis.at<float>(0,0)=yVertex.x-origin.x;
	yAxis.at<float>(0,1)=yVertex.y-origin.y;
	Mat xAxis(1,2,DataType<float>::type);
	xAxis.at<float>(0,0)=xVertex.x-origin.x;
	xAxis.at<float>(0,1)=xVertex.y-origin.y;
	Mat pointVector(1,2,DataType<float>::type);
	pointVector.at<float>(0,0)=p.x-origin.x;
	pointVector.at<float>(0,1)=p.y-origin.y;

	double yProjection=pointVector.dot(yAxis);
	double xProjection=pointVector.dot(xAxis);
	double yNorm=yAxis.dot(yAxis);
	double xNorm=xAxis.dot(xAxis);
	return yProjection>=0 && yNorm>=yProjection && xProjection>=0 && xNorm>=xProjection;
}

Hole getHole(vector<Point> circonferenza){
	//center of mass to find the circle center
	float centerX=0;
	float centerY=0;
	for(size_t i=0; i<circonferenza.size(); i++){
		centerX+=circonferenza[i].x;
		centerY+=circonferenza[i].y;
	}
	centerX=centerX/circonferenza.size();
	centerY=centerY/circonferenza.size();
	Point center(centerX,centerY);

	//averaged radius
	float radius=0;
	for(size_t i=0; i<circonferenza.size(); i++)
	{
		radius+=getDistance(center,circonferenza[i],EUCLIDIAN);
	}
	radius=radius/circonferenza.size();
	return Hole(center,radius);
}

vector<Rods> detachRods(Mat image, vector<Hole> holes, Rect ROI, bool isFast){
	vector<Rods> result;

	Mat detachedBlob=detachBlob(image);

	// Create the CV_8U version of the distance image,needed by findContours
	Mat detach_8u;
	detachedBlob.convertTo(detach_8u, CV_8U);

	// Find contours
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	findContours(detach_8u, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	for( size_t i = 0; i < contours.size(); i++ )
	{
		RotatedRect rr = minAreaRect( Mat(contours[i]) );
		Rods r=computeApproximatedRod(image,rr,holes);
		if(isFast){
			r.applyTranslation(ROI.x,ROI.y);
			r.ROI=ROI;
			r.area=-1;
			r.approximated=true;
		}
		result.push_back(r);
	}
	if(isFast){
		return result;
	} else {
		//label the image using the approximated measurement
		applyLabel(&image,result);

		//for each labeled component made further analysis
		for(size_t i=0; i<result.size(); i++){
			//create a image with only one labeled item
			Mat lookUpTable(1, 256, CV_8U);
			uchar* p = lookUpTable.data;
			for( size_t k = 0; k < 256; k++){
				if(k==(i+1))
					p[k] = 255;
				else
					p[k]=0;
			}
			Mat workingCopy(image.rows,image.cols,image.type());
			LUT(image,lookUpTable,workingCopy);

			//find contours and enclosing rectangle
			vector<vector<Point>> c;
			vector<Vec4i> h;
			vector<RotatedRect> boundingBoxes=findContoursAndBoxes(workingCopy,c,h);

			for( size_t k = 0; k < c.size(); k++ )
			{
				//compute final rod
				if(h[k][3]==-1){
					result[i].contours=c[k];
					result[i].enclosingRectangle=boundingBoxes[k];
					result[i].applyTranslation(ROI.x,ROI.y);
					result[i].ROI=ROI;
					result[i].approximated=true;
					break;
				}
			}
		}

		return result;
	}
}

Mat detachBlob(Mat image){
	// Erode a bit
	Mat temp;
	Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1));
	erode(image, temp, kernel);

	// Perform the distance transform algorithm
	Mat dist;
	distanceTransform(temp, dist, CV_DIST_L2, 5);

	// Normalize the distance image for range = {0.0, 1.0} to threshold it
	normalize(dist, dist, 0, 1., NORM_MINMAX);

	// Threshold to obtain the foreground objects
	threshold(dist, dist, .5, 1., CV_THRESH_BINARY);

	//dilate to make rods bigger
	dilate(dist,dist,kernel);

	return dist;
}

Rods computeApproximatedRod(Mat image, RotatedRect boundingBox, vector<Hole> holes){
	//create an initial guess for the rods
	Rods r;
	r.enclosingRectangle=boundingBox;
	r.baricenter=boundingBox.center;

	//find the holes that has the center near to the major axis of the approximated rod
	int soglia=10;
	for(size_t i=0; i<holes.size(); i++){
		float distance=r.getDistanceFromMajorAxis(holes[i].center);
		if(distance<soglia){
			r.holes.push_back(holes[i]);
		}
	}

	float angCoeff=r.getMajorAxisAngularcoefficient();

	//find holes thickeness
	for(size_t i=0; i<r.holes.size(); i++){
		Point2f a=findFrontChange(image,r.holes[i].center,angCoeff,r.holes[i].center.x,image.cols,SCAN_X,2);
		Point2f b=findFrontChange(image,r.holes[i].center,angCoeff,r.holes[i].center.x,0,SCAN_X,2);
		Point2f c=findFrontChange(image,r.holes[i].center,-angCoeff,r.holes[i].center.y,0,SCAN_Y,2);
		Point2f d=findFrontChange(image,r.holes[i].center,-angCoeff,r.holes[i].center.y,image.rows,SCAN_Y,2);

		int distA=getDistance(r.holes[i].center,a,EUCLIDIAN);
		int distB=getDistance(r.holes[i].center,b,EUCLIDIAN);
		int distC=getDistance(r.holes[i].center,c,EUCLIDIAN);
		int distD=getDistance(r.holes[i].center,d,EUCLIDIAN);

		//check if the distance perpendicular to the major axis are nearly the same
		if(distC-distD<5)
			r.holes[i].spessore=((float)distC+distD)/2-r.holes[i].radius;
		//otherways pick the smallest value
		else if(distA<=distB && distA<=distC && distA<=distD)
			r.holes[i].spessore=distA-r.holes[i].radius;
		else if(distB<=distC && distB<=distD)
			r.holes[i].spessore=distB-r.holes[i].radius;
		else if(distC<=distD)
			r.holes[i].spessore=distC-r.holes[i].radius;
		else
			r.holes[i].spessore=distD-r.holes[i].radius;
	}

	//build the enclosing rotated rect with some aproximation
	RotatedRect rect;
	Size2f s;
	float versore[2];
	r.getVersorMajorAxis(versore);
	if(r.holes.size()==1){
		//rods is of type A
		Point extreme;
		if(boundingBox.center.x>r.holes[0].center.x)
			extreme=findFrontChange(image,boundingBox.center,angCoeff,boundingBox.center.x,image.cols,SCAN_X,1);
		else
			extreme=findFrontChange(image,boundingBox.center,angCoeff,boundingBox.center.x,0,SCAN_X,1);
		s.height=r.holes[0].radius*2+2*r.holes[0].spessore;
		s.width=getDistance(r.holes[0].center,extreme,EUCLIDIAN)+r.holes[0].radius+r.holes[0].spessore;
		Point center;
		if(r.holes[0].center.y>=boundingBox.center.y){
			center.x=r.holes[0].center.x+(s.width/2-r.holes[0].spessore-r.holes[0].radius)*versore[0];
			center.y=r.holes[0].center.y+(s.width/2-r.holes[0].spessore-r.holes[0].radius)*versore[1];
		} else {
			center.x=r.holes[0].center.x-(s.width/2-r.holes[0].spessore-r.holes[0].radius)*versore[0];
			center.y=r.holes[0].center.y-(s.width/2-r.holes[0].spessore-r.holes[0].radius)*versore[1];
		}
		rect.center=center;
		rect.size=s;
		rect.angle=boundingBox.angle;
	} else {
		//rods is of type B
		Hole bigHole=(r.holes[0].radius>r.holes[1].radius)?r.holes[0]:r.holes[1];
		s.height=bigHole.radius*2+2*bigHole.spessore;
		s.width=getDistance(r.holes[0].center, r.holes[1].center,EUCLIDIAN)+r.holes[0].radius+r.holes[1].radius+r.holes[0].spessore+r.holes[1].spessore;
		Point center((r.holes[0].center.x+r.holes[1].center.x)/2,(r.holes[0].center.y+r.holes[1].center.y)/2);
		rect.center=center;
		rect.size=s;
		rect.angle=boundingBox.angle;
	}

	r.enclosingRectangle=rect;
	r.baricenter=rect.center;
	return r;
}

Point2f findFrontChange(Mat image, Point2f p, float angCoeff,int start, int end, directionType mode, int nChange){
	/* Scan the image along a given line of "angCoeff" angularCoefficient and passing for p
	 *	from "start" to "end" searching for "nChange" intensity changes,
	 *	then returns the point were the last change happened*/
	Point2f result;
	int x,y,lastx,lasty;
	int lastInt=image.at<uchar>(p);
	int currInt=lastInt;
	if(mode==SCAN_X){
		//scan along x
		if(start<end){
			for(x=start; x<end; x++){
				y=angCoeff*(x-p.x)+p.y;
				currInt=image.at<uchar>(y,x);
				if(currInt!=lastInt){
					nChange--;
					if(nChange==0)
						break;
				}
				lastInt=currInt;
				lasty=y;
			}
			result.x=x--;
			result.y=lasty;
		} else {
			for(x=start; x>end; x--){
				y=angCoeff*(x-p.x)+p.y;
				currInt=image.at<uchar>(y,x);
				if(currInt!=lastInt){
					nChange--;
					if(nChange==0)
						break;
				}
				lastInt=currInt;
				lasty=y;
			}
			result.x=x++;
			result.y=lasty;
		}
	}else {
		//scan along y
		if(start<end){
			for(y=start; y<end; y++){
				x=angCoeff*(y-p.y)+p.x;
				currInt=image.at<uchar>(y,x);
				if(currInt!=lastInt){
					nChange--;
					if(nChange==0)
						break;
				}
				lastInt=currInt;
				lastx=x;
			}
			result.x=lastx;
			result.y=y--;
		}else {
			for(y=start; y>end; y--){
				x=angCoeff*(y-p.y)+p.x;
				currInt=image.at<uchar>(y,x);
				if(currInt!=lastInt){
					nChange--;
					if(nChange==0)
						break;
				}
				lastInt=currInt;
				lastx=x;
			}
			result.x=lastx;
			result.y=y++;
		}
	}
	if(nChange!=0){
		result.x=-1;
		result.y=-1;
	}
	return result;
}

void applyLabel(Mat*image,vector<Rods> rods){
	int inside[rods.size()];
	for(int y=0; y<image->rows; y++){
		for(size_t x=0; x<image->step; x++){

			//check if is a foreground pixel
			if(image->at<uchar>(y,x)!=0){
				//reset inside array
				for(size_t i=0; i<rods.size(); i++){
					inside[i]=0;
				}

				//check if is inside a rotated rectangle
				for(size_t i=0; i<rods.size(); i++){
					if(isInside(Point(x,y), rods[i].enclosingRectangle))
						inside[i]=1;
				}
				int somma=0;
				int label;
				for(size_t i=0; i<rods.size(); i++){
					somma+=inside[i];
					if(inside[i]!=0)
						label=(i+1);
				}

				if(somma==1){
					//the point is inside a certain enclosing rectangle so apply its label
					image->at<uchar>(y,x)=label;
					rods[label-1].area++;
				} else if(somma==0){
					//the point is not inside a certain enclosing rectangle
					//search in the neighborhood(8 connectivity) of the point if there's some labeled pixel
					bool stop=false;
					for(int i=-1; i<2 && !stop; i++){
						for(int j=-1; j<2 && !stop; j++){
							int value=image->at<uchar>(y+i,x+j);
							if(value!=255 && value!=0){
								image->at<uchar>(y,x)=value;
								rods[value-1].area++;
								stop=true;
							}
						}
					}
					//otherways mark it as background
					if(!stop)
						image->at<uchar>(y,x)=0;
				} else {
					//the point is inside two or more enclosing rectangle!
					bool stop=false;
					//check if is inside one of the circonference
					for(size_t i=0; i<rods.size() && !stop; i++){ //check if is inside a circle
						if(inside[i]==1){
							for(size_t k=0; k<rods[i].holes.size() && !stop; k++ ){
								if(rods[i].holes[k].isInside(Point(x,y))){
									image->at<uchar>(y,x)=(i+1);
									rods[i].area++;
									stop=true;
								}
							}
						}
					}
					if(!stop){
						//otherways label it according to its distance from the major axis of the rods
						float distance[rods.size()];
						for(size_t i=0; i<rods.size(); i++)
							distance[i]=10000;
						for(size_t i=0; i<rods.size(); i++){
							if(inside[i]==1){
								distance[i]=rods[i].getDistanceFromMajorAxis(Point(x,y));
							}
						}
						int min=10000;
						int minIndex=-1;
						for(size_t i=0; i<rods.size(); i++){
							if(distance[i]<min){
								min=distance[i];
								minIndex=(i+1);
							}
						}
						image->at<uchar>(y,x)=minIndex;
						rods[minIndex-1].area++;
					}
				}
			}
		}

	}
}
