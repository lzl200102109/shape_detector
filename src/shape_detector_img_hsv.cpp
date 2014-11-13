#include "feature.h"
#include "geometry.h"

using namespace cv;
using namespace std;



// main function.
int main(int argc, char** argv) {

	// create control window
	CreateControlWindow();

	// image processing
	Mat imgOriginal;
	Mat imgThresholded;
	Mat imgTracked;
	Mat_<double> imagePts;

	while (true) {
		imgOriginal = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

		if(! imgOriginal.data )                              // Check for invalid input
		{
			cout <<  "Could not open or find the image" << endl ;
			return -1;
		}

//		cout << imgOriginal.cols << "," << imgOriginal.rows << endl;

		imagePts = detectCentroids(imgOriginal);

		cout << imagePts << endl;
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
