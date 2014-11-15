#include "feature.h"
using namespace cv;
using namespace std;

// trackbar
int threshold_value = 100;
int threshold_type = 1;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;
int const morph_size = 5;

void setLabel(Mat& image, Point2f center, string shape_label) {
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.3;
	int thickness = 0.5;
	int baseline = 0;

	ostringstream x_s, y_s;
	x_s << center.x;
	y_s << center.y;
	string label = shape_label + "(" + x_s.str() + "," + y_s.str() + ")";

	Size text = getTextSize(label, fontface, scale, thickness, &baseline);


	Point pt(center.x - text.width/2,
			 center.y + text.height/2);

	rectangle(image, pt + Point(0, baseline),
			pt + Point(text.width, -text.height), CV_RGB(255, 255, 255),
			CV_FILLED);
	putText(image, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

void ThresholdImage(Mat &imgOriginal, Mat &imgGrayscale, Mat &imgThresholded) {


	/// create trackbar to choose type of threshold
	namedWindow("Threshold Control", CV_WINDOW_AUTOSIZE);
	createTrackbar( "Threshold Type", "Threshold Control", &threshold_type, max_type);
	createTrackbar( "Threshold Value", "Threshold Control", &threshold_value, max_value);

	// convert to grayscale image
	cvtColor(imgOriginal,imgGrayscale,CV_BGR2GRAY);

	/* 0: Binary
	 * 1: Binary Inverted
	 * 2: Threshold Truncated
	 * 3: Threshold to Zero
	 * 4: Threshold to Zero Inverted
	 */
	threshold( imgGrayscale, imgThresholded, threshold_value, max_BINARY_value,threshold_type );

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(morph_size, morph_size)));
	dilate(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(morph_size, morph_size)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(morph_size, morph_size)));
	erode(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(morph_size, morph_size)));
}

Mat getFeatureVector(Mat &imgThresholded, Mat &imgFeature) {

	/* imagePts stores the image coord's in the following order:
	 * 0: large circle
	 * 1: small circle
	 * 2: square
	 * 3. rectangle
	 * 4. triangle
	 * 5. hexagon
	 */
	Mat_<double> imagePts(6,3);
	imagePts << 0,0,0,
			    0,0,0,
			    0,0,0,
			    0,0,0,
			    0,0,0,
			    0,0,0;					// image coordinates of detected shapes.

//	cout << imagePts << endl;

	//finding all contours in the image
	vector<Vec4i> hierarchy;  					// hold the pointer to a contour
	vector<vector<Point> > contours; 			// storage area for all contours
	findContours(imgThresholded, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly( contours.size() );
	vector<double> area( contours.size() );
	vector<RotatedRect> minRect( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );
	vector<string> shape_label( contours.size() );

	int j = 0;	// circle index

	for( int i = 0; i < contours.size(); i++ ) {

		// approximate polygns.
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		// Skip small or non-convex objects
		if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(contours_poly[i]))
			continue;

		area[i] = contourArea(contours[i]);
		minRect[i] = minAreaRect( Mat(contours_poly[i]) );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );

		double L = minRect[i].size.width;
		double H = minRect[i].size.height;
		double r = radius[i];
		double S = area[i];
		double x = center[i].x;
		double y = center[i].y;
		double pi = CV_PI;

		if  (	// detect circle
					(0.8 < L/H && L/H < 1.2)
				 && (L*H > S)
				 && (pi*pow(r,2) - S < 0.5*pow(r,2))
				 && (40 < x && x < 600)
				 && (40 < y && y < 440)
			) {
			shape_label[i] = "Cir";

			imagePts(j,0) = x;
			imagePts(j,1) = y;
			imagePts(j,2) = 1;

			// re-order the circle index if necessary.
			double r_temp[2] = {0, 0};
			r_temp[j] = r;
			if (j == 1 && r_temp[0] < r_temp[1]) {
				imagePts(1,0) = imagePts(0,0);
				imagePts(1,1) = imagePts(0,1);
				imagePts(1,2) = imagePts(0,2);
				imagePts(0,0) = x;
				imagePts(0,1) = y;
				imagePts(0,2) = 1;
			}
			j++;

		} else if (	// detect square
					(0.8 < L/H && L/H < 1.2)
				 && (L*H -S < 0.2*L*H)
				 && (L*H > (pi-1.5)*pow(r,2))
				 && (40 < x && x < 600)
				 && (40 < y && y < 440)
				  ) {
			shape_label[i] = "Sqr";
			imagePts(2,0) = x;
			imagePts(2,1) = y;
			imagePts(2,2) = 1;

		} else if (	// detect rectangle
					(L/H > 2 || H/L > 2)
				 && (pi*pow(r,2) - L*H > 0.2*pow(r,2))
				 && (L*H - S < 0.2*L*H)
				 && (40 < x && x < 600)
				 && (40 < y && y < 440)
				  ) {
			shape_label[i] = "Rec";
			imagePts(3,0) = x;
			imagePts(3,1) = y;
			imagePts(3,2) = 1;

		} else if (	// detect triangle
					(L*H > 1.5*S)
				 && (L*H < pi*pow(r,2))
				 && (40 < x && x < 600)
				 && (40 < y && y < 440)
				  ) {
			shape_label[i] = "Tri";
			imagePts(4,0) = x;
			imagePts(4,1) = y;
			imagePts(4,2) = 1;

		} else if (	// detect hexagon
					(L/H < 2 || H/L < 2)
				 && (L*H > 0.9*pi*pow(r,2))
				 && (pi*pow(r,2) - S > 0)
				 && (40 < x && x < 600)
				 && (40 < y && y < 440)
				  ) {
			shape_label[i] = "Hex";
			imagePts(5,0) = x;
			imagePts(5,1) = y;
			imagePts(5,2) = 1;

		}else {
			ostringstream i_s;
			i_s << i;
			shape_label[i] = " ";		// i_s.str() +
		}

		// generate feature on the original image.
		if (shape_label[i] != " ") {
			setLabel(imgFeature, center[i], shape_label[i]);
			drawContours( imgFeature, contours_poly, i, cvScalar(255,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
			Point2f rect_points[4];
			circle( imgFeature, center[i], (int)radius[i], cvScalar(0,255, 0), 1, 8, 0 );
			minRect[i].points( rect_points );
			for( int j = 0; j < 4; j++ ) {
				line( imgFeature, rect_points[j], rect_points[(j+1)%4], cvScalar(255,0,0), 1, 8 );
			}
		}
	 }

//	cout << imagePts << endl << endl;
	return imagePts;
}
