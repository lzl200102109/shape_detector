#include "feature.h"
#include "geometry.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

// images.
Mat imgOriginal;
Mat imgGrayscale;
Mat imgThresholded;
Mat imgFeature;

// function prototypes.
void imageCallback(const sensor_msgs::ImageConstPtr& );

// main function.
int main(int argc, char** argv) {

	// initialize ROS.
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // subscribe webcam video from usb_cam.
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &imageCallback);

    // define windows for image display.
    namedWindow("Original Image");
    namedWindow("Grayscale Image");
    namedWindow("Thresholded Image");
    namedWindow("Feature Image");
    startWindowThread();

    ros::spin();

    return 0;
}

// function definitions.

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
    imgGrayscale = imgOriginal.clone();
    imgThresholded = imgOriginal.clone();
    imgFeature = imgOriginal.clone();

    // threshold original image.
	ThresholdImage(imgOriginal, imgGrayscale, imgThresholded);

	// get feature vector
	getFeatureVector(imgThresholded, imgFeature);

	// show images.
	imshow("Original Image", input_bridge->image); 	// show the original image
	imshow("Grayscale Image", imgGrayscale);		// show the grayscale image
	imshow("Thresholded Image", imgThresholded); 	// show the thresholded image
	imshow("Feature Image", imgFeature); 			// show the original image

	// Press ESC to exit.
	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	{
		cout << "ESC key is pressed by user" << endl;
		imwrite( "landing_pad.jpg", imgOriginal );
		ros::shutdown();
	}
}
