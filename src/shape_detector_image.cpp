#include "feature.h"
#include "geometry.h"
using namespace cv;
using namespace std;

// main function.
int main(int argc, char** argv) {

	// image processing
	Mat imgOriginal;
	Mat imgGrayscale;
	Mat imgThresholded;
	Mat imgFeature;
	Mat imgTracked;

	// image processing
	while (true) {

		// load the image
		imgOriginal = imread(argv[1], CV_LOAD_IMAGE_COLOR);
		imgFeature = imgOriginal;
		if(! imgOriginal.data )
		{
			cout <<  "Could not open or find the image" << std::endl ;
			return -1;
		}

		// threshold original image.
		 ThresholdImage(imgOriginal, imgGrayscale, imgThresholded);

		 // get feature vector
		 getFeatureVector(imgThresholded, imgFeature);

		// show images.
		imshow("Original Image", imgOriginal); //show the original image
		imshow("Grayscale Image", imgGrayscale); //show the original image
		imshow("Thresholded Image", imgThresholded); //show the thresholded image

		// Press ESC to exit.
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	return 0;
}
