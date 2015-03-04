#include "inc/function.hpp"
#include "inc/Hole.hpp"
#include "inc/Rod.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/utility.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define THRESHOLD_RECTANGULAR 30
#define THRESHOLD_DISTANCE 10
#define THRESHOLD_AREA 5500

/** @function main */
int main( int argc, char** argv )
{
	Mat sourceImage;

	string originalWindowsName="Original Image";
	string outputWindowNAme="Results";

	while(true){
		//choose image to load
		cout<<"Welcome to Motorcycle Rods analysis tool!"<<endl<<"Enter fileName to continue or END to exit: ";
		string filename;
		cin>>filename;
		if(filename=="END"){
			return 0;
		}

		//load the source image
		sourceImage=imread(filename, 0);
		// Check for invalid input
		if(!sourceImage.data )
		{
			cout <<  "Could not open or find the image: '" <<filename<<"'"<< endl ;
			continue;
		}

		//show the loaded image
		namedWindow(originalWindowsName);
		imshow(originalWindowsName, sourceImage);
		cout<<"Press any key to start the analysis..."<<endl;
		waitKey(0);

		//performance test
		clock_t    start;
		start = clock();

		//Median Filter x3 to remove salt and pepper noise --> larger kernel will corrupt the image
		Mat blurredImage(sourceImage.rows,sourceImage.cols,sourceImage.type());
		medianBlur(sourceImage,blurredImage,3);
		medianBlur(blurredImage,blurredImage,3);
		medianBlur(blurredImage,blurredImage,3);
		//imshow(thresholdingWindowName,blurredImage);
		//waitKey(0);


		//perform segmentation with otsu algorithm
		Mat thresholdImage;
		threshold( blurredImage, thresholdImage, 0, 255,THRESH_BINARY_INV|THRESH_OTSU );
		//threshold( sourceImage, thresholdImage, 0, 255,THRESH_BINARY_INV|THRESH_OTSU );

		//find connected components
		Mat labeledImage;
		Mat labeledNormalized;
		Mat stats;
		Mat centroids;
		int nLabels=connectedComponentsWithStats(thresholdImage, labeledImage, stats, centroids, 8, CV_16U);

		//find oriented bounding boxes and contours in hierarchical order
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		Mat thresholdCopy;
		thresholdImage.copyTo(thresholdCopy);
		vector<RotatedRect> boundingBoxes=findContoursAndBoxes(thresholdCopy,contours,hierarchy);

		//imshow(thresholdingWindowName, thresholdImage);
		//waitKey(0);

		//build rods array from the labeled components
		vector<Rods> rods;
		for(int i=1; i<nLabels; i++){
			Rods rod;

			//information obtained from the labeling algorithm
			Point origin(centroids.at<double>(i,0),centroids.at<double>(i,1));
			Rect roi(stats.at<int>(i,CC_STAT_LEFT),stats.at<int>(i,CC_STAT_TOP),stats.at<int>(i,CC_STAT_WIDTH),stats.at<int>(i,CC_STAT_HEIGHT));
			rod.area=stats.at<int>(i,CC_STAT_AREA);
			rod.baricenter=origin;
			rod.ROI=roi;
			rod.approximated=false;

			//check if the blob is composed of more than one rod
			if(rod.area>THRESHOLD_AREA){
				//build array of all the holes int he new reference system
				vector<Hole> holes;
				for(size_t k=0; k<hierarchy.size(); k++){
					if(hierarchy[k][3]!=-1){
						Hole temp=getHole(contours[k]);
						temp.center.x-=rod.ROI.x;
						temp.center.y-=rod.ROI.y;
						holes.push_back(temp);
					}
				}
				vector<Rods> temp=detachRods(thresholdImage(rod.ROI),holes, rod.ROI,false);
				for(size_t j=0; j<temp.size(); j++){
					rods.push_back(temp[j]);
				}
				continue;
			}


			//find the boundingBox for this specific rod
			size_t index;
			for( index=0; index<boundingBoxes.size(); index++){
				Point boxCenter(boundingBoxes[index].center.x,boundingBoxes[index].center.y);
				if(getDistance(rod.baricenter,boxCenter ,EUCLIDIAN)<THRESHOLD_DISTANCE)
					break;
			}
			rod.enclosingRectangle=boundingBoxes[index];

			//check whatever the bounding box is rectangular or not, if it's not skip this labeled item
			if(abs(rod.enclosingRectangle.size.width-rod.enclosingRectangle.size.height)<THRESHOLD_RECTANGULAR)
				continue;

			//find the circle(s) inside the choosen bounding box
			for(size_t i=0; i<hierarchy.size(); i++){
				//in hierarchy [i][3] is saved the parent of the i-th contours
				if(hierarchy[i][3]==index){
					rod.holes.push_back(getHole(contours[i]));
				}
			}

			//if the labeled item doesn't contain holes skip it
			if(rod.holes.size()==0)
				continue;

			rods.push_back(rod);
		}

		//remove duplicates rods
		for(size_t i=0; i<rods.size(); i++ ){
			for(size_t k=i+1; k<rods.size(); k++){
				if(rods[i]==rods[k]){
					if(rods[i].approximated)
						rods.erase(rods.begin()+i);
					else
						rods.erase(rods.begin()+k);
				}
			}
		}

		//normalize the labeled image for visualization purpose
		normalize(labeledImage, labeledNormalized, 0, 200, NORM_MINMAX, CV_8UC1);

		for(size_t i=0; i<rods.size();i++){

			//compute width at baricenter and draw it
			Point rightCorner;
			Point leftCorner;
			rods[i].widthAtBaricenter(&labeledNormalized, &rightCorner, &leftCorner);
			line(labeledNormalized, leftCorner, rightCorner, Scalar(255,255,255),1,8);

			//draw rod's baricenter togheter with label
			circle(labeledNormalized,rods[i].baricenter,5,Scalar(0,0,0),-1);
			stringstream name;
			name<<"Rods "<<i;
			putText(labeledNormalized,name.str(), rods[i].baricenter,FONT_HERSHEY_COMPLEX_SMALL,0.5,Scalar(255,255,255),1);

			//draw minimum enclosing rotated rectangle
			Point2f rect_points[4];
			rods[i].enclosingRectangle.points( rect_points );
			for( int j = 0; j < 4; j++ )
				line( labeledNormalized, rect_points[j], rect_points[(j+1)%4], Scalar(255,255,255), 1, 8 );

			//draw circle center and radius
			for(size_t j=0; j<rods[i].holes.size(); j++){
				circle(labeledNormalized,rods[i].holes[j].center,3,Scalar(255,255,255),-1);
				Point radius(rods[i].holes[j].center.x,rods[i].holes[j].center.y-rods[i].holes[j].radius);
				line(labeledNormalized, rods[i].holes[j].center, radius,Scalar(255,255,255),1,8);
			}

			//print statistics
			cout<<"Rod n° "<<i<<" stats: "<<endl;
			cout<<rods[i].getDescription()<<endl;
			cout<<"-----------------------------------------------------------"<<endl;

		}

		//show final image
		namedWindow(outputWindowNAme);
		imshow(outputWindowNAme,labeledNormalized);

		//print performance statistic
		std::cout << "Time: " << (clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << endl;
		cout<<"Press any key to start a new analysis"<<endl;
		waitKey(0);

		//close all windows
		destroyAllWindows();
	}
}
/*End Main Function*/

