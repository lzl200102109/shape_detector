#include <math.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

// trackbar

int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;
int const morph_size = 1;

// function prototypes.

static double angle(Point , Point , Point );

void setLabel(Mat& , Point2f, string );

void ThresholdImage(Mat &, Mat &, Mat &);

Mat getFeatureVector(Mat &, Mat &);
