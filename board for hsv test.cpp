#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;


/*
NOTES:
BGR ile daha güzel görüntü alınıyor.
HoughCircles parametreleri ayarlanırsa olacak gibi.


*/

Mat image;
int thresh = 100;
int max_thresh = 255;
char* image_window = "Source Image";
char* hue_low = "Hue low";
char* hue_high = "Hue high";
char* val_low = "Val low";
char* val_high = "Val high";
char* sat_low = "Sat low";
char* sat_high = "Sat high";
int lowhue = 0;
int highhue = 255;
int lowsat = 0;
int highsat = 255;
int lowval = 0;
int highval = 255;


Mat RGBBLUE(Mat);
Mat FindWhiteTriangles(Mat);
Mat FindWhiteTriangles1(Mat);
void templateMatching(Mat img);
void HSV_detect( int, void* );

/** @function main */
int main( int argc, char** argv )
{
  VideoCapture Cap;
  if (!Cap.open(0)) 
    return 0;
  while (true){
  Cap >> image;
  

  createTrackbar( hue_low,  image_window, &lowhue, 179, HSV_detect );
  createTrackbar( hue_high, image_window, &highhue, 179, HSV_detect );
  createTrackbar( sat_low,  image_window, &lowsat, 255, HSV_detect );
  createTrackbar( sat_high, image_window, &highsat, 255, HSV_detect );
  createTrackbar( val_low,  image_window, &lowval, 255, HSV_detect );
  createTrackbar( val_high, image_window, &highval, 255, HSV_detect );
  HSV_detect( 0, 0 );


  waitKey(0);

  }
  return 0;
  
}


Mat RGBBLUE(Mat input) {
	Mat image = input.clone();
	
	for (int i=0; i < image.rows; i++){
		for (int j=0; j < image.cols; j++){
			Point3_<uchar>* p = image.ptr<Point3_<uchar> >(i,j);
			if (p->x > 240){ p->x = 255;}
			if (p->x > 250 and p->y > 200 or p->z > 200){p->x = 0;}
			p->y = 0;
			p->z = 0;
			//waitKey(50);
		}
	}
	
	cvtColor(image, image, COLOR_BGR2GRAY);
	Mat erode_term = getStructuringElement(MORPH_RECT, Size (5,1) );
  	erode(image, image, erode_term, Point(-1, -1), 3);


	double thresh =15;
  	double max = 255;
  	threshold(image, image, thresh, max, THRESH_BINARY);
  	//GaussianBlur( image, image, Size(9, 9), 2, 2 );
    return image;
}


/*Mat FindWhiteTriangles(Mat input){
  Mat image = input.clone();
  
  for (int i=0; i < image.rows; i++){
    for (int j=0; j < image.cols; j++){
      Point3_<uchar>* p = image.ptr<Point3_<uchar> >(i,j);
      if (p->x > 100 and p->y > 100 and p->z > 100){p->x = 255;p->y = 255;p->z = 255;}
      else {p->x = 0;p->y = 0;p->z = 0;}
      //waitKey(50);
    }
  }
  Mat x = RGBBLUE(input);
  Mat result;
  bitwise_xor(image, x, result);
  imshow("result",result);
  


  cvtColor(image, image, COLOR_BGR2GRAY);
  //Mat erode_term = getStructuringElement(MORPH_RECT, Size (5,1) );
  //  erode(image, image, erode_term, Point(-1, -1), 3);
  bitwise_xor(image, x, result);
  imshow("result",result);
    double thresh =15;
    double max = 255;
  //  threshold(image, image, thresh, max, THRESH_BINARY);
    GaussianBlur( image, image, Size(9, 9), 2, 2 );

  return image;
}
*/


Mat FindWhiteTriangles1(Mat input){
  Mat image = input.clone();
  

  cvtColor(image, image, COLOR_BGR2GRAY);
  //inRange(image, Scalar(0,0,200), Scalar(180, 255, 255), image );
  double thresh =230;
  double max = 255;
  threshold(image, image, thresh, max, THRESH_TOZERO); //THRESH_BINARY
  GaussianBlur( image, image, Size(9, 9), 2, 2 );
  return image;
}

void templateMatching(Mat img) 
{
  Mat result;
  Mat result_1;
  Mat result_end;

  Mat templ = imread("triangle1.png", CV_LOAD_IMAGE_COLOR);   // Read the file

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  //result.create( result_rows, result_cols, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, CV_TM_CCORR_NORMED );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  threshold(result, result, 0.77, 1, THRESH_TOZERO);
  imshow("result", result);

  /*
  matchTemplate( img, templ, result_1, CV_TM_CCOEFF_NORMED );
  normalize( result_1, result_1, 0, 1, NORM_MINMAX, -1, Mat() );
  threshold(result_1, result_1, 0.70, 1, THRESH_TOZERO);
  imshow("result_1", result_1);

  bitwise_and(result, result_1, result_end);
  imshow("result_end", result_end);
  */
  //testWindow(result,"matching");
  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  matchLoc = maxLoc;
  //technique is
  /// Show me what you got
  rectangle( img, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  //imshow("1",img);
  //imshow("2",result);
}

void HSV_detect( int, void* )
{
  Mat result;
  Mat image_1 = image.clone();
  cvtColor(image_1, result, cv::COLOR_BGR2HSV);
  inRange(result, cv::Scalar(lowhue, lowsat, lowval), cv::Scalar(highhue, highsat, highval), result);
  imshow(image_window, result);
  return;
}