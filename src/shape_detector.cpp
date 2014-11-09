#include <iostream>
#include <ros/ros.h>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "Geometry.h"
using namespace cv;
using namespace std;


// define constants.
int iLowH = 0;
int iHighH = 179;

int iLowS = 220;
int iHighS = 255;

int iLowV = 70;
int iHighV = 255;

// trackbar
int threshold_value = 60;
int threshold_type = 0;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

// images.
Mat imgOriginal;
Mat imgGrayscale;
Mat imgThresholded;
Mat imgTracked;

// function prototypes.

// Helper function to find a cosine of angle between vectors from pt0->pt1 and pt0->pt2
static double angle(Point , Point , Point );

// Helper function to display text in the center of a contour
void setLabel(Mat& , const string , vector<Point>& );

void ThresholdImage(Mat &, Mat &, Mat &);

void ShapeIdentification(Mat &, Mat &, Mat &);

void imageCallback(const sensor_msgs::ImageConstPtr& );


// main function.
int main(int argc, char** argv) {

	// create control window.
//	CreateControlWindow();

	// initialize ROS.
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // subscribe webcam video from usb_cam.
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &imageCallback);

    namedWindow("Original Image");
    namedWindow("Grayscale Image");
    namedWindow("Thresholded Image");
    namedWindow("Tracked Image");
    startWindowThread();

    ros::spin();

    return 0;
}

// function definitions.
static double angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2)
			/ sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void setLabel(Mat& im, const string label, vector<Point>& contour) {
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	Size text = getTextSize(label, fontface, scale, thickness, &baseline);
	Rect r = boundingRect(contour);

	Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

void ThresholdImage(Mat &imgOriginal, Mat &imgGrayscale, Mat &imgThresholded)
{
	/// create trackbar to choose type of threshold
	namedWindow("Threshold Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	createTrackbar( "Threshold Type", "Threshold Control", &threshold_type, max_type);
	createTrackbar( "Threshold Value", "Threshold Control", &threshold_value, max_value);

	// convert to grayscale image
	cvtColor(imgOriginal,imgGrayscale,CV_BGR2GRAY);

	/* 0: Binary
	   1: Binary Inverted
	   2: Threshold Truncated
	   3: Threshold to Zero
	   4: Threshold to Zero Inverted
	*/
	threshold( imgGrayscale, imgThresholded, threshold_value, max_BINARY_value,threshold_type );

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
	dilate(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
	erode(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
}

void ShapeIdentification(Mat &imgOriginal, Mat &imgThresholded, Mat &imgTracked) {

	vector<Vec4i> hierarchy;  //hold the pointer to a contour
	CvSeq* result;   //hold sequence of points of a contour
	vector<vector<Point> > contours; //storage area for all contours

	imgTracked = imgOriginal.clone();


	//finding all contours in the image
	findContours(imgThresholded, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	vector<Point> approx;
	string label;
//	Mat_<double> imgPts(6,2);
	int j = 0;		// img point index
//	cout << contours.size() << endl;
	for (int i = 0; i < contours.size(); i++) { //

		// Approximate contour with accuracy proportional to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

		// Skip small or non-convex objects
		if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
			continue;

		if (approx.size() == 3) {

//			average(approx);
			ostringstream x_s, y_s;
			double x=1.0, y=1.0;
			x_s << x;
			y_s << y;
			label = "TRI(" + x_s.str() + "," + y_s.str() + ")";
			setLabel(imgTracked, label, contours[i]);    // Triangles
			//drawing lines around the triangle
			line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
			line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
			line(imgTracked, approx[2], approx[0], cvScalar(255,0,0),4);

		}
		else if (approx.size() >= 4 && approx.size() <= 6) {
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			vector<double> cos;
			for (int j = 2; j < vtc + 1; j++)
				cos.push_back(angle(approx[j % vtc], approx[j - 2], approx[j - 1]));

			// Sort ascending the cosine values
			sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
				setLabel(imgTracked, "RECT", contours[i]);
				//drawing lines around the triangle
				line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
				line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
				line(imgTracked, approx[2], approx[3], cvScalar(255,0,0),4);
				line(imgTracked, approx[3], approx[0], cvScalar(255,0,0),4);
			}
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27) {
				setLabel(imgTracked, "PENTA", contours[i]);
				//drawing lines around the triangle
				line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
				line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
				line(imgTracked, approx[2], approx[3], cvScalar(255,0,0),4);
				line(imgTracked, approx[3], approx[4], cvScalar(255,0,0),4);
				line(imgTracked, approx[4], approx[0], cvScalar(255,0,0),4);
			}
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45) {
				setLabel(imgTracked, "HEXA", contours[i]);
				//drawing lines around the triangle
				line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
				line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
				line(imgTracked, approx[2], approx[3], cvScalar(255,0,0),4);
				line(imgTracked, approx[3], approx[4], cvScalar(255,0,0),4);
				line(imgTracked, approx[4], approx[5], cvScalar(255,0,0),4);
				line(imgTracked, approx[5], approx[0], cvScalar(255,0,0),4);
			}
		}
		else {
			// Detect and label circles
			double area = contourArea(contours[i]);
			Rect r = boundingRect(contours[i]);
			int radius = r.width / 2;

			if (abs(1 - ((double) r.width / r.height)) <= 0.2
					&& abs(1 - (area / (CV_PI * pow(radius, 2))))
							<= 0.2) {
				setLabel(imgTracked, "CIR", contours[i]);

				//drawing lines around the circle
				vector<Point2f>center(1);
				vector<float>radius(1);
				minEnclosingCircle( (Mat)approx, center[0], radius[0] );
				circle( imgTracked, center[0], (int)radius[0], cvScalar(255,0,0), 2, 8, 0 );

			}

		}
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // convert the ROS image message to opencv IplImage
    cv_bridge::CvImagePtr input_bridge;
    try{
        input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR16);
    }
    catch (cv_bridge::Exception& ex) {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }

    // get the original image.
    imgOriginal = input_bridge->image;

    // threshold original image.
	 ThresholdImage(imgOriginal, imgGrayscale, imgThresholded);

	// identify shapes.
	 ShapeIdentification(imgOriginal, imgThresholded, imgTracked);

	// show images.
	imshow("Original Image", input_bridge->image); //show the original image
	imshow("Grayscale Image", imgGrayscale);
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	imshow("Tracked Image", imgTracked); //show the original image


	// Press ESC to exit.
	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	{
		cout << "ESC key is pressed by user" << endl;
		imwrite( "landing_pad.jpg", imgOriginal );
		ros::shutdown();
	}

}
