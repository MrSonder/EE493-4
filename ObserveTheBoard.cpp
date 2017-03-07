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
int hue_low = 62;
int hue_high = 112;
int sat_low = 35;
int sat_high = 95;
int val_low = 230;
int val_high = 255;


Mat RGBBLUE(Mat);
Mat FindWhiteTriangles(Mat);
Mat FindWhiteTriangles1(Mat);
void templateMatching(Mat img);
void HSV_detect( int, void* );
Mat DrawLines(Mat input);

/** @function main */
int main( int argc, char** argv )
{
  VideoCapture Cap;
  if (!Cap.open(0)) 
    return 0;
  while (true){
  Cap >> image;
  

  Mat x = FindWhiteTriangles(image);
  imshow("x",x);
  imshow("image",image);

  int a = waitKey(10);
  if(a==32) {imwrite("saved.jpg",image);imwrite("saved_x.jpg",x);break;}

  }
  return 0;
  
}


Mat DrawLines(Mat input){
  Mat image = input.clone();
  
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