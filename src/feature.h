#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;



// functioon prototypes.

Mat_<double> detectCentroids(Mat &);

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(Point , Point , Point );

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(Mat& , const string , vector<Point>&);

void setLabel(Mat& , Point2f );

void CreateControlWindow();

void ThresholdImage(Mat &, Mat &);

void ThresholdImage(Mat &, Mat &, Mat &);

void ShapeIdentification(Mat &, Mat &, Mat &);

Point calCentroid (vector<Point> & );

void getFeatureVector(Mat &);
