#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;


/*
NOTES:


*/

Mat image;
int thresh = 100;
int max_thresh = 255;
int hue_low = 62;
int hue_high = 112;
int sat_low = 35;
int sat_high = 95;
int val_low = 230;
int val_high = 255;

Mat FindCorners(Mat input, Mat test);
Mat FindWhiteTriangles(Mat);
void templateMatching(Mat img);
void HSV_detect( int, void* );
Mat DrawLines(Mat input);
Mat FillInside(Mat input);

/** @function main */
int main( int argc, char** argv )
{
  image = imread("board_latest.jpg", CV_LOAD_IMAGE_COLOR);
  

  Mat x = FindWhiteTriangles(image);
  imshow("x",x);
  Mat y = FillInside(x);
  DrawLines(y);

  int a = waitKey(0);
  //if(a==32) {imwrite("saved.jpg",image);imwrite("saved_x.jpg",x);break;}
  
  return 0;
  
}


Mat FindCorners(Mat input, Mat test){
	
	int length = input.checkVector(2);

	int xmin = 0, xmax = 0, ymin = 0;
	Point ptxmin, ptxmax, ptymin;

	const Point* pts = input.ptr<Point>();
	Point pt = pts[0];

	ptxmin  = ptxmax = ptymin = pt;
    xmin = xmax = pt.x;
    ymin = pt.y;

    for( int i = 1; i < length; i++ )
    {
        pt = pts[i];

        if( xmin > pt.x )
        {
            xmin = pt.x;
            ptxmin = pt;
        }


        if( xmax < pt.x )
        {
            xmax = pt.x;
            ptxmax = pt;
        }

        if( ymin > pt.y )
        {
            ymin = pt.y;
            ptymin = pt;
        }
    }
    Point testing = Point((ptxmax.x+ptxmin.x)/2.0,(ptxmax.y+ptxmin.y)/2.0);
    
    // Determine where the end point is going to be for the lines.
    // For ptxmax, ptymin line:
    double slope = double(ptxmax.y - ptymin.y)/(ptxmax.x - ptymin.x);
    cout << slope;
    double slope_to_zero = double(ptxmax.y - 0)/(ptxmax.x - 0);
	cout << slope_to_zero<<endl;
	// If slope_to_zero is smaller than slope, it means that line is going to hit y=0 axis.
	// Else, it hits x= 0 axis.
	if (slope_to_zero<slope){
		Point edge = Point(int(-(ptxmax.y/slope)+ptxmax.x) ,0);
		cout<<"hehe"<<edge<<endl;
		line( test, ptxmax, edge, Scalar( 255, 0, 0 ),  2, 8 );
	}
	else{
		Point edge = Point(0, int(-slope*ptxmax.x+ptxmax.y));
		line( test, ptxmax, edge, Scalar( 255, 0, 0 ),  2, 8 );	

	}


	// For ptxmin, ptymin line:
	double slope = double(ptxmax.y - ptymin.y)/(ptxmax.x - ptymin.x);
    cout << slope;
    double slope_to_zero = double(ptxmax.y - 0)/(ptxmax.x - 0);
	cout << slope_to_zero<<endl;

    
    line( test, ptxmin, ptymin, Scalar( 255, 0, 0 ),  2, 8 );

    return test;
}


Mat FillInside(Mat input){
  Mat image = input.clone();
  Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*4 + 1, 2*4+1 ), Point( 4, 4 ) );
  Mat result;
  morphologyEx( image, result, MORPH_CLOSE, element );
  return result;
}

Mat DrawLines(Mat input){
  Mat image = input.clone();
  blur( image, image, Size(3,3) );
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int thresh = 100;

  Canny( image, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  Mat test = Mat::zeros(image.size(), image.type());

  for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    drawing = FindCorners(Mat(contours[i]), drawing);
    }
  
  waitKey(0);
  cout<<image.size()<<endl;
  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  return drawing;
}


Mat FindWhiteTriangles(Mat input){
  Mat image = input.clone();
  Mat image_hsv;
  Mat image_mask;

  cvtColor(image, image_hsv, COLOR_BGR2HSV);
  inRange(image_hsv, Scalar(hue_low,sat_low,val_low), Scalar(hue_high, sat_high, val_high), image_mask );
  double thresh =230;
  double max = 255;
  threshold(image_mask, image_mask, thresh, max, THRESH_TOZERO); //THRESH_BINARY
  //GaussianBlur( image_mask, image_mask, Size(9, 9), 2, 2 );
  return image_mask;
}





void HSV_detect( int, void* ){
  Mat result;
  Mat image_1 = image.clone();
  cvtColor(image_1, result, cv::COLOR_BGR2HSV);
  inRange(result, Scalar(hue_low,sat_low,val_low), Scalar(hue_high, sat_high, val_high), result);
  imshow("y", result);
  return;
}