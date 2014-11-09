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

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr&);

// images.
Mat imgOriginal;

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

    ros::spin();

    return 0;
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

	// show images.
	imshow("Original Image", input_bridge->image); //show the original image

	// Press ESC to exit.
	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	{
		imwrite( "landing_pad.jpg", imgOriginal );
		ros::shutdown();
	}

}
