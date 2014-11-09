#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


// define constants.
int iLowH = 0;
int iHighH = 179;

int iLowS = 227;
int iHighS = 255;

int iLowV = 51;
int iHighV = 255;

// functioon prototypes.

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(Point , Point , Point );

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(Mat& , const string , vector<Point>&);

void CreateControlWindow();

void ThresholdImage(Mat &, Mat &);

void ShapeIdentification(Mat &, Mat &, Mat &);

Point calCentroid (vector<Point> & points) {
	Point centroid = Point(0,0);
	for (int i = 0; i < points.size(); i++)
		centroid = centroid + points[i];
	centroid.x = centroid.x / points.size();
	centroid.y = centroid.y / points.size();
	return centroid;
}

// main function.
int main(int argc, char** argv) {

	// create control window
	CreateControlWindow();

	// image processing
	Mat imgOriginal;
	Mat imgThresholded;
	Mat imgTracked;
	while (true) {
		imgOriginal = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

		if(! imgOriginal.data )                              // Check for invalid input
		{
			cout <<  "Could not open or find the image" << endl ;
			return -1;
		}

		cout << imgOriginal.cols << "," << imgOriginal.rows << endl;

		// threshold original image.
		 ThresholdImage(imgOriginal, imgThresholded);

		// identify shapes.
		 ShapeIdentification(imgOriginal, imgThresholded, imgTracked);

		// show images.
		imshow("Original", imgOriginal); //show the original image
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Tracked Image", imgTracked); //show the original image

		// Press ESC to exit.
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

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

void setLabel(Mat& im, string label, vector<Point>& approx) {
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.3;
	int thickness = .5;
	int baseline = 0;

	Point centroid = calCentroid(approx);		// centroid
	ostringstream x_s, y_s;
	x_s << centroid.x;
	y_s << centroid.y;
	label = label + "(" + x_s.str() + "," + y_s.str() + ")";

	Size text = getTextSize(label, fontface, scale, thickness, &baseline);


	Point pt(centroid.x - text.width/2,
			 centroid.y + text.height/2);

	rectangle(im, pt + Point(0, baseline),
			pt + Point(text.width, -text.height), CV_RGB(255, 255, 255),
			CV_FILLED);
	putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

void CreateControlWindow() {

	namedWindow("Control"); //create a window called "Control"     , CV_WINDOW_AUTOSIZE

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);
}

void ThresholdImage(Mat &imgOriginal, Mat &imgThresholded)
{
	Mat imgHSV;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

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
}

void ShapeIdentification(Mat &imgOriginal, Mat &imgThresholded, Mat &imgTracked) {

	vector<Vec4i> hierarchy;  //hold the pointer to a contour
	CvSeq* result;   //hold sequence of points of a contour
	vector<vector<Point> > contours; //storage area for all contours

	//finding all contours in the image
	findContours(imgThresholded, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	Point centroid;
	vector<Point> approx;
	string label;
	imgTracked = imgOriginal.clone();
//	cout << contours.size() << endl;

	for (int i = 0; i < contours.size(); i++) {

		// Approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx,
				arcLength(Mat(contours[i]), true) * 0.02, true);

		// Skip small or non-convex objects
		if (fabs(contourArea(contours[i])) < 100
				|| !isContourConvex(approx))
			continue;

		if (approx.size() == 3) {
//			centroid = calCentroid(approx);
//			cout << contours[i] << endl;
			setLabel(imgTracked, "TRI", approx);    // Triangles
			//drawing lines around the triangle
			line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
			line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
			line(imgTracked, approx[2], approx[0], cvScalar(255,0,0),4);


//			cout << approx[0] << "," << approx[1] << "," << approx[2] << "," << centroid[0] << endl;
		}
		else if (approx.size() >= 4 && approx.size() <= 6) {
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			vector<double> cos;
			for (int j = 2; j < vtc + 1; j++)
				cos.push_back(
						angle(approx[j % vtc], approx[j - 2], approx[j - 1]));

			// Sort ascending the cosine values
			sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
				setLabel(imgTracked, "RECT", approx);    // Rectangles
				//drawing lines around the triangle
				line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
				line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
				line(imgTracked, approx[2], approx[3], cvScalar(255,0,0),4);
				line(imgTracked, approx[3], approx[0], cvScalar(255,0,0),4);
			}
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27) {
				setLabel(imgTracked, "PENTA", approx);
				//drawing lines around the triangle
				line(imgTracked, approx[0], approx[1], cvScalar(255,0,0),4);
				line(imgTracked, approx[1], approx[2], cvScalar(255,0,0),4);
				line(imgTracked, approx[2], approx[3], cvScalar(255,0,0),4);
				line(imgTracked, approx[3], approx[4], cvScalar(255,0,0),4);
				line(imgTracked, approx[4], approx[0], cvScalar(255,0,0),4);
			}
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45) {
				setLabel(imgTracked, "HEXA", approx);
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
				setLabel(imgTracked, "CIR",approx);
				//drawing lines around the circle
	//					Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				vector<Point2f>center(1);
				vector<float>radius(1);
				minEnclosingCircle( (Mat)approx, center[0], radius[0] );
				circle( imgTracked, center[0], (int)radius[0], cvScalar(255,0,0), 2, 8, 0 );

			}

		}
	}
}
