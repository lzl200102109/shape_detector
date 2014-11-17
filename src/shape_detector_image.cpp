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

		bool method = 1;	// default method is by simple geometry.
		if (method) {
			/* METHOD 1: SIMPLE GEOMETRY */

			// get image points
			Mat_<double> imagePts = getFeatureVector(imgThresholded, imgFeature);
			imagePts = calibrateImagePoints(getFeatureVector(imgThresholded, imgFeature));
//			cout << "imgPts = " << endl << imagePts << endl << endl;

			// get world points
			Mat_<double> worldPts = getWorldPts();
//			cout << "worldPts = " << endl << worldPts << endl << endl;

			// estimate pose
			Mat_<double> simplePose = estimatePose_GEO(imagePts, worldPts);
//			cout << "simplePose = " << endl << simplePose.t() << endl;

		} else {
			/* METHOD 2: SIGULAR VALUE DECOMPOSITION*/

			// get image points
			Mat_<double> imagePts = calibrateImagePoints(getFeatureVector(imgThresholded, imgFeature));
//			cout << "imgPts = " << endl << imagePts << endl << endl;

			// get world points
			Mat_<double> worldPts = getWorldPts();

			cout << "imgPts = " << endl << imagePts << endl << endl;
//			cout << "worldPts = " << endl << worldPts << endl << endl;


			// estimate pose
			Mat_<double> simplePose = estimatePose_SVD(imagePts, worldPts);
//			cout << "simplePose = " << endl << simplePose.t() << endl;

		}


		// show images.
//		imshow("Original Image", imgOriginal); //show the original image
		imshow("Grayscale Image", imgGrayscale); //show the original image
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Featured Image", imgFeature); //show the thresholded image

		// Press ESC to exit.
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	return 0;
}
