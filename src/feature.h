#include <math.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

// function prototypes.

static double angle(Point , Point , Point );

void setLabel(Mat& , Point2f, string );

void ThresholdImage(Mat &, Mat &, Mat &);

Mat getFeatureVector(Mat &, Mat &);
