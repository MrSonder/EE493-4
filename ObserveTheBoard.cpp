#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;


/*
NOTES:
Açık uçlu contourlar silinmeli.
Contours sıralanması gerekecek.
En alttaki satır da çizilmeli.

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
Mat FindAreas(Mat input);
Mat FillInside(Mat input);

/** @function main */
int main( int argc, char** argv )
{
  image = imread("board_latest.jpg", CV_LOAD_IMAGE_COLOR);
  

  Mat x = FindWhiteTriangles(image);
  //imshow("x",x);
  Mat y = FillInside(x);
  Mat result;
  result = DrawLines(y);
  imshow( "Contours", result );
  Mat z;
  z = FindAreas(result);
  imshow( "Mine", z );


  int a = waitKey(0);
  if(a==32) {imwrite("for_report2.jpg",z);}
  //if (a == 27) break;
  
  return 0;
  
}


Mat FindCorners(Mat input, Mat test){
	/*
	Draws necessary connections.
	NOTE: Lower ones need to be drawn.
	*/
	
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

    double slope_to_zero = double(ptxmax.y - 0)/(ptxmax.x - 0);

	// If slope_to_zero is smaller than slope, it means that line is going to hit y=0 axis.
	// Else, it hits x= 0 axis.
	if (slope_to_zero<slope){
		Point edge = Point(int(-(ptxmax.y/slope)+ptxmax.x) ,0);
		
		line( test, ptxmax, edge, Scalar( 255, 0, 0 ),  2, 8 );
	}
	else{
		Point edge = Point(0, int(-slope*ptxmax.x+ptxmax.y));
		line( test, ptxmax, edge, Scalar( 255, 0, 0 ),  2, 8 );	

	}


	// For ptxmin, ptymin line:
	slope = double(ptxmin.y - ptymin.y)/(ptymin.x - ptxmin.x);

    slope_to_zero = double(ptxmin.y - 0)/(640 - ptxmin.x);


	// If slope_to_zero is smaller than slope, it means that line is going to hit y=0 axis.
	// Else, it hits x= 640 axis.

	if (slope_to_zero<slope){
		Point edge = Point(int((ptxmin.y/slope)+ptxmin.x) ,0);
		
		line( test, ptxmin, edge, Scalar( 255, 0, 0 ),  2, 8 );
	}
	else{
		Point edge = Point(640, int(slope*(ptxmin.x - 640)+ptxmin.y));
		line( test, ptxmin, edge, Scalar( 255, 0, 0 ),  2, 8 );	

	}
    
    

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

  /*
  vector<Moments> mu(contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }


  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }


  vector<Point> mass_center( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
	{ mass_center[i] = Point( int(mc[i].x),int(mc[i].y) ); }  		
  */


  for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    if (hierarchy[i][3] != -1){continue;}
    //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    //if (hierarchy[i][3] != -1){continue;}
    drawing = FindCorners(Mat(contours[i]), drawing);

	// Need to eliminate contours on top of each other.


    
    }
  
  

  /// Show in a window
  //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  //imshow( "Contours", drawing );
  return drawing;
}

Mat FindAreas(Mat input){
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
    if (hierarchy[i][3] != -1){continue;}
    if (contourArea(contours[i])<4000){continue;}
    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

    }
  
  

  /// Show in a window
  //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  //imshow( "Contours", drawing );
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