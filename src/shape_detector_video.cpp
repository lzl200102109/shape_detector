#include "feature.h"
#include "geometry.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
//#include <opencv/cvwimage.h>
//#include <opencv/highgui.h>

using namespace cv;
using namespace std;
using namespace std_msgs;

// images.
Mat imgOriginal;
Mat imgGrayscale;
Mat imgThresholded;
Mat imgFeature;

static Mat_<double> simplePose(7,1);
static Mat_<double> simplePose_pre(7,1);
static bool pose_valid = false;
ros::Publisher simplePosePub;
image_transport::Publisher imgFeaturePub;

// function prototypes.
void imageCallback(const sensor_msgs::ImageConstPtr& );

Float64MultiArray makeSimplePoseMsg(Mat_<double> const );

// main function.
int main(int argc, char** argv) {

	// initialize ROS.
    ros::init(argc, argv, "shape_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    int const queueSize = 100;

//    ros::Rate loopRate(1);

    // "simplePose" message: [x, y, z, yaw] -- contains the estimate
    // of the camera pose w.r.t. the landing pad
    simplePosePub = nh.advertise<Float64MultiArray>("simplePose", queueSize);
    imgFeaturePub = it.advertise("camera/image_feature", 1);

	// define windows for image display.
//	namedWindow("Original Image");
//	namedWindow("Grayscale Image");
//	namedWindow("Thresholded Image");
//	namedWindow("Feature Image");
//	startWindowThread();

	// subscribe webcam video from usb_cam.
	image_transport::Subscriber sub = it.subscribe("/camera/image_rect", 1, &imageCallback);

	ros::spin();



	return 0;
}

// function definitions.

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ros::Rate loopRate(25);

    // convert the ROS image message to opencv IplImage
    cv_bridge::CvImagePtr input_bridge_original;
    try{
    	input_bridge_original = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR16);
    }
    catch (cv_bridge::Exception& ex) {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }
    // get the original image.
    imgOriginal = input_bridge_original->image;
    imgGrayscale = imgOriginal.clone();
    imgThresholded = imgOriginal.clone();
    imgFeature = imgOriginal.clone();

    // threshold original image.
	ThresholdImage(imgOriginal, imgGrayscale, imgThresholded);

	bool method = 0;	// default method is by simple geometry.
	if (method) {
			/* METHOD 1: SIMPLE GEOMETRY */

			// get image points
			Mat_<double> imagePts = calibrateImagePoints(getFeatureVector(imgThresholded, imgFeature));

			// get world points
			Mat_<double> worldPts = getWorldPts();

//				cout << "imgPts = " << endl << imagePts << endl << endl;
//				cout << "worldPts = " << endl << worldPts << endl << endl;

			// estimate pose
			simplePose = estimatePose_GEO(imagePts, worldPts);
//			cout << "simplePose = " << endl << simplePose.t() << endl;

	} else {
			/* METHOD 2: SIGULAR VALUE DECOMPOSITION*/

			// get image points
			Mat_<double> imagePts = calibrateImagePoints(getFeatureVector(imgThresholded, imgFeature));

			// get world points
			Mat_<double> worldPts = getWorldPts();

//				cout << "imgPts = " << endl << imagePts << endl << endl;
//				cout << "worldPts = " << endl << worldPts << endl << endl;

			// estimate pose ( in millimeter )
			simplePose = estimatePose_SVD(imagePts, worldPts, simplePose_pre, pose_valid); 		//
			simplePose_pre = simplePose;
//			cout << "simplePose = " << endl << simplePose.t() << endl;

	}

//	simplePose = LPF(simplePose_pre, simplePose);
//	simplePose_pre = simplePose;

	// publish image with detected features.
	cv_bridge::CvImagePtr input_bridge_feature(new cv_bridge::CvImage);
	input_bridge_feature->image = imgFeature;
	sensor_msgs::ImagePtr imgFeatureMsg = input_bridge_feature->toImageMsg();
	imgFeatureMsg->encoding = "mono8";
	imgFeaturePub.publish(msg);

	// send pose estimate only when it is valid.
	if (pose_valid) {
		// send pose estimate message ( in meter )
		Float64MultiArray simplePoseMsg = makeSimplePoseMsg(simplePose);
		simplePosePub.publish(simplePoseMsg);
	}

	// show images.
//	imshow("Original Image", input_bridge->image); 	// show the original image
//	imshow("Grayscale Image", imgGrayscale);		// show the grayscale image
//	imshow("Thresholded Image", imgThresholded); 	// show the thresholded image
//	imshow("Feature Image", imgFeature); 			// show the original image

	// Press ESC to exit.
	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	{
		cout << "ESC key is pressed by user" << endl;
		ros::shutdown();
	}


	loopRate.sleep();
//	ros::spinOnce();

}

Float64MultiArray makeSimplePoseMsg(Mat_<double> const simplePose)
{
    Float64MultiArray simplePoseMsg;	// in meter
    simplePoseMsg.data.push_back(simplePose(0)/1000);
    simplePoseMsg.data.push_back(simplePose(1)/1000);
    simplePoseMsg.data.push_back(simplePose(2)/1000);
    simplePoseMsg.data.push_back(simplePose(3)/1000);
    simplePoseMsg.data.push_back(simplePose(4)/1000);
    simplePoseMsg.data.push_back(simplePose(5)/1000);
    simplePoseMsg.data.push_back(simplePose(6));
    return simplePoseMsg;
}
