#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2)
			/ sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label,
		std::vector<cv::Point>& contour) {
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness,
			&baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2),
			r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline),
			pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255),
			CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}




//////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {







	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 220;
	int iHighS = 255;

	int iLowV = 70;
	int iHighV = 255;

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	int iLastX = -1;
	int iLastY = -1;

	//Capture a temporary image from the camera
	Mat imgTmp;
	Mat imgTracked;
	cap.read(imgTmp);

	while (true) {
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV),
				Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (removes small holes from the foreground)
		dilate(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

















		vector<Vec4i> hierarchy;  //hold the pointer to a contour
		CvSeq* result;   //hold sequence of points of a contour
		vector<vector<Point> > contours; //storage area for all contours

		//finding all contours in the image
		findContours(imgThresholded, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

		vector<Point> approx;
		Mat imgTracked = imgOriginal.clone();

		for (int i = 0; i < contours.size(); i++) {
			// Approximate contour with accuracy proportional
			// to the contour perimeter
			approxPolyDP(Mat(contours[i]), approx,
					arcLength(Mat(contours[i]), true) * 0.02, true);

			// Skip small or non-convex objects
			if (std::fabs(contourArea(contours[i])) < 100
					|| !isContourConvex(approx))
				continue;

			if (approx.size() == 3) {
				setLabel(imgTracked, "TRI", contours[i]);    // Triangles
				//drawing lines around the triangle
				line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
				line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
				line(imgTracked, approx[2], approx[0], cvScalar(255,0,0),4);

			}
			else if (approx.size() >= 4 && approx.size() <= 6) {
				// Number of vertices of polygonal curve
				int vtc = approx.size();

				// Get the cosines of all corners
				std::vector<double> cos;
				for (int j = 2; j < vtc + 1; j++)
					cos.push_back(
							angle(approx[j % vtc], approx[j - 2], approx[j - 1]));

				// Sort ascending the cosine values
				std::sort(cos.begin(), cos.end());

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

				if (std::abs(1 - ((double) r.width / r.height)) <= 0.2
						&& std::abs(1 - (area / (CV_PI * std::pow(radius, 2))))
								<= 0.2) {
					setLabel(imgTracked, "CIR", contours[i]);
					//drawing lines around the circle
//					Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
					vector<Point2f>center(1);
					vector<float>radius(1);
					minEnclosingCircle( (Mat)approx, center[0], radius[0] );
					circle( imgTracked, center[0], (int)radius[0], cvScalar(255,0,0), 2, 8, 0 );

				}

			}
		}





















		imshow("Original", imgOriginal); //show the original image
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Tracked Image", imgTracked); //show the original image

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;
}

