#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;


/*
NOTES:
Sıralama tamam. 
Contourların içinin okunup bir datastructure'a atılması kaldı.


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

Mat FindCorners(Mat input, Mat test, int control);
Mat FindWhiteTriangles(Mat);
void templateMatching(Mat img);
void HSV_detect( int, void* );
Mat DrawLines(Mat input);
Mat FindAreas(Mat input, Mat original_image);
Mat FillInside(Mat input);
Point GetMassCenter(Mat input);
Point GetRightLeg(Mat input);
Point GetLeftLeg(Mat input);
bool WhatColor(Mat image, Mat input, int color_choice);


/** @function main */
int main( int argc, char** argv )
{
  image = imread("board_latest.jpg", CV_LOAD_IMAGE_COLOR);
  

  Mat x = FindWhiteTriangles(image);
  //imshow("x",x);
  Mat y = FillInside(x);
  Mat result;
  result = DrawLines(y);
  //imshow( "Contours", result );
  Mat z;
  imshow("Board", image);
  z = FindAreas(result, image);
  imshow( "Decision", z );


  int a = waitKey(0);
  if(a==32) {imwrite("for_report.jpg",z);}
  //if (a == 27) break;
  
  return 0;
  
}


Mat FindCorners(Mat input, Mat test, int control){
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
	if (control != -1) {
	if (slope_to_zero<slope){
		Point edge = Point(int(-(ptxmax.y/slope)+ptxmax.x) ,0);
		
		line( test, ptxmax, edge, Scalar( 255, 0, 0 ),  2, 8 );
	}
	else{
		Point edge = Point(0, int(-slope*ptxmax.x+ptxmax.y));
		line( test, ptxmax, edge, Scalar( 255, 0, 0 ),  2, 8 );	

	}
	}
	else {line( test, ptxmax, ptymin, Scalar( 255, 0, 0 ),  2, 8 );	}

	// For ptxmin, ptymin line:
	slope = double(ptxmin.y - ptymin.y)/(ptymin.x - ptxmin.x);

    slope_to_zero = double(ptxmin.y - 0)/(640 - ptxmin.x);


	// If slope_to_zero is smaller than slope, it means that line is going to hit y=0 axis.
	// Else, it hits x= 640 axis.
    if (control != 1) {
	if (slope_to_zero<slope){
		Point edge = Point(int((ptxmin.y/slope)+ptxmin.x) ,5); //5 -> 0
		
		line( test, ptxmin, edge, Scalar( 255, 0, 0 ),  2, 8 );
	}
	else{
		Point edge = Point(640, int(slope*(ptxmin.x - 630)+ptxmin.y)); //630 -> 640
		line( test, ptxmin, edge, Scalar( 255, 0, 0 ),  2, 8 );	

	}
    }
    else {line( test, ptxmin, ptymin, Scalar( 255, 0, 0 ),  2, 8 );	}
    

    return test;
}

Point GetMassCenter(Mat input){
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
    Point masscenter = Point((ptxmax.x+ptxmin.x+ptymin.x)/3,(ptxmax.y+ptxmin.y+ptymin.y)/3);
    
    return masscenter;
}

Point GetRightLeg(Mat input){
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

        if( xmax < pt.x )
        {
            xmax = pt.x;
            ptxmax = pt;
        }

        
    }
    
    return ptxmax;
}
Point GetLeftLeg(Mat input){
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

    }
   
    return ptxmin;
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

  int number_of_contours = 0;
  for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    if (hierarchy[i][3] != -1){continue;}
    number_of_contours++;
    //drawing = FindCorners(Mat(contours[i]), drawing);
	
    }
    int counter = 0;
    // Holds contours whom are actually drawn.
    int array_contours [number_of_contours] ;
    int contour_centers [number_of_contours] ;
    for( int i = 0; i< contours.size(); i++ ){
    if (hierarchy[i][3] != -1){continue;}
    array_contours[counter] = i;
    contour_centers[counter] = GetMassCenter(Mat(contours[i])).x ;
    
    counter++;
    }
    
    int ranking [number_of_contours];
    for( int i = 0; i<number_of_contours; i++ ){
    	int counter = 0;
    	for( int j = 0; j< number_of_contours; j++ ){
    		if (contour_centers[i]>contour_centers[j]){
    			counter++;
    		}
    	}
    	ranking[i] = counter;
    	
    }
  	

    int sorted_contours [number_of_contours];
    for(int i=0; i<number_of_contours; i++ ){
    	sorted_contours[ranking[i]] = array_contours[i];
    }

    Point right_leg;
    Point left_leg;
    for( int i = 0; i< number_of_contours; i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    int control = 0;
    if (i == 0) control=-1;
    if (i == number_of_contours-1) control=1;
    drawing = FindCorners(Mat(contours[sorted_contours[i]]), drawing, control);
    if (i>0) right_leg = GetRightLeg(Mat(contours[sorted_contours[i-1]]));
    if (i>0 ) left_leg = GetLeftLeg(Mat(contours[sorted_contours[i]]));

    if (i>0) line( drawing, left_leg, right_leg, Scalar( 255, 0, 0 ),  2, 8 );
	}	

  return drawing;
}

Mat FindAreas(Mat input, Mat original_image){
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


  // Ranking from left to right will be done here.
  int number_of_contours=0;
  for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    if (hierarchy[i][3] != -1){continue;}
    if (contourArea(contours[i])<4000){continue;}
    number_of_contours++;
    //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    //cout<<contourArea(contours[i])<<endl;
    //imshow("sadasd", drawing);
    //waitKey(0);
    }

  vector<Point> mass_centers(number_of_contours) ;
  int miny [number_of_contours] = {0};
  int neutral_ranking [number_of_contours];
  int counter =0;
  int counter_for_miny =0;
  for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    if (hierarchy[i][3] != -1){continue;}
    if (contourArea(contours[i])<4000){continue;}
    mass_centers[counter] = GetMassCenter(Mat(contours[i]));
    neutral_ranking[counter] = i;
    
    //cout << counter<<"\t"<< mass_centers[counter]<<endl;
    if (counter_for_miny == 0) {miny[counter_for_miny] = mass_centers[counter].y; counter_for_miny++;}
    else {
    	for (int j =0; j<counter_for_miny; j++){
    		// The line below makes closer y's considered to be equal.
    		if (abs(mass_centers[counter].y-miny[j])<30) {mass_centers[counter] = Point(mass_centers[counter].x,miny[j]);break;}
    		if (j == counter_for_miny-1) {miny[counter_for_miny] = mass_centers[counter].y; counter_for_miny++;}
    		}
    	}

    
    counter++;
    }
   // mass_centers[number_of_contours] still present and updated now.
   int sorted_contours [number_of_contours];
   counter = 0;
   for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar( 0, 0, 255 );
    if (hierarchy[i][3] != -1){continue;}
    if (contourArea(contours[i])<4000){continue;}
    int counter_for_rank = 0;
    for (int j = 0; j < number_of_contours; j++){
    	if ((mass_centers[counter].x> mass_centers[j].x and mass_centers[counter].y== mass_centers[j].y) or mass_centers[counter].y< mass_centers[j].y) {counter_for_rank++;}

    	}
    //sorted_contours[counter] = counter_for_rank;
    sorted_contours[counter_for_rank] = i;
    //cout << counter << "\t"<< mass_centers[counter]<< "\t"<< counter_for_rank<<  endl;	
    counter++;
    }
   Mat final = Mat::zeros( canny_output.size(), CV_8UC3 );
   counter = 0;
   for( int i = 0; i< number_of_contours; i++ ){
    Scalar color = Scalar( 255, 255, 255 );
    Scalar color_red = Scalar( 0, 0, 255 );
    Scalar color_blue = Scalar( 255, 0, 0 );
    //if (hierarchy[i][3] != -1){continue;}
    //if (contourArea(contours[i])<4000){continue;}
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    drawContours( drawing, contours, sorted_contours[i], color, -1, 8, hierarchy, 0, Point() );

    //cout<<contourArea(contours[i])<<endl;
    //imshow("sadasd", drawing);
    
    if (WhatColor(original_image,drawing, 0)){
    cout << "RED" << endl;
    drawContours( final, contours, sorted_contours[i], color_red, -1, 8, hierarchy, 0, Point() );
    }
    else if (WhatColor(original_image,drawing, 1)){
	cout << "BLUE" << endl;
	drawContours( final, contours, sorted_contours[i], color_blue, -1, 8, hierarchy, 0, Point() );
    }
    else {cout << "EMPTY" << endl;
    drawContours( final, contours, sorted_contours[i], color, -1, 8, hierarchy, 0, Point() );}
    
    }


  
  

  /// Show in a window
  //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  //imshow( "Contours", drawing );
  return final;
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

bool WhatColor(Mat image, Mat input, int color_choice){
	Mat image_hsv = image.clone();
	cvtColor(input, input, COLOR_BGR2GRAY);
	//imshow("aw312", input);
	// Color choice = 0 -> RED, Color choice = 1 -> BLUE
	// 
	int hue_low_red = 111;
	int hue_high_red = 179;
	int sat_low_red = 116;
	int sat_high_red = 255;
	int val_low_red = 167;
	int val_high_red = 255;

	int hue_low_blue = 9;
	int hue_high_blue = 179;
	int sat_low_blue = 237;
	int sat_high_blue = 255;
	int val_low_blue = 165;
	int val_high_blue = 255;

	int decision = 2000; // This is the threshold for colors.
	//Mat drawing = Mat::zeros( image.size(), CV_8UC3 );

	cvtColor(image_hsv, image_hsv, COLOR_BGR2HSV);
  	
	if (color_choice==0) {
		inRange(image_hsv, Scalar(hue_low_red,sat_low_red,val_low_red), Scalar(hue_high_red, sat_high_red, val_high_red), image_hsv );
		double thresh =230;
  		double max = 255;
  		threshold(image_hsv, image_hsv, thresh, max, THRESH_BINARY); //THRESH_BINARY
  		bitwise_and(image_hsv,input,image_hsv);
  		int counter = 0;
  		for (int i=0; i<image_hsv.rows; i++) {
  			for (int j=0; j<image_hsv.cols; j++){

  				if (image_hsv.at<uchar>(i,j) == 255 ) {
  				counter++;

  				}
  			}
		}
		//cout << counter << endl;
		if (counter > decision) return true;
	}
	else if(color_choice == 1){
		inRange(image_hsv, Scalar(hue_low_blue,sat_low_blue,val_low_blue), Scalar(hue_high_blue, sat_high_blue, val_high_blue), image_hsv );
		double thresh =230;
  		double max = 255;
  		threshold(image_hsv, image_hsv, thresh, max, THRESH_BINARY); //THRESH_BINARY
  		bitwise_and(image_hsv,input,image_hsv);
  		int counter = 0;
  		for (int i=0; i<image_hsv.rows; i++) {
  			for (int j=0; j<image_hsv.cols; j++){

  				if (image_hsv.at<uchar>(i,j) == 255 ) {
  				counter++;

  				}
  			}
		}
		//cout << counter << endl;
		if (counter > decision) return true;	

		}
	return false;
}




void HSV_detect( int, void* ){
  Mat result;
  Mat image_1 = image.clone();
  cvtColor(image_1, result, cv::COLOR_BGR2HSV);
  inRange(result, Scalar(hue_low,sat_low,val_low), Scalar(hue_high, sat_high, val_high), result);
  imshow("y", result);
  return;
}
