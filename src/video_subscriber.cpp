#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using namespace std;
using namespace cv;

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
    imshow("Image window", input_bridge->image);
//    waitKey(3);
	// Press ESC to exit.
	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	{
		cout << "ESC key is pressed by user" << endl;
		ros::shutdown();
	}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &imageCallback);

    namedWindow("Image window");
    startWindowThread();

    ros::spin();
}
