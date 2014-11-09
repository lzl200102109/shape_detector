//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main()
{

	IplImage* img =  cvLoadImage("/home/zhilong/Documents/ObjectDetectionContours/test1/FindingContours.png");

	//show the original image
	cvNamedWindow("Raw");
	cvShowImage("Raw",img);

	//converting the original image into grayscale
	IplImage* imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1);
	cvCvtColor(img,imgGrayScale,CV_BGR2GRAY);

	//thresholding the grayscale image to get better results
	cvThreshold(imgGrayScale,imgGrayScale,128,255,CV_THRESH_BINARY);

	CvSeq* contour;  //hold the pointer to a contour
	CvSeq* result;   //hold sequence of points of a contour
	CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

	//finding all contours in the image
	cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	//iterating through each contour
	while(contour)
	{
		//obtain a sequence of points of the countour, pointed by the variable 'countour'
		result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*0.02, 0);

		//if there are 3 vertices  in the contour(It should be a triangle)
		if(result->total==3 )
		{
			//iterating through each point
			CvPoint *pt[3];
			for(int i=0;i<3;i++){
				pt[i] = (CvPoint*)cvGetSeqElem(result, i);
			}

			//drawing lines around the triangle
			cvLine(img, *pt[0], *pt[1], cvScalar(255,0,0),4);
			cvLine(img, *pt[1], *pt[2], cvScalar(255,0,0),4);
			cvLine(img, *pt[2], *pt[0], cvScalar(255,0,0),4);

		}

		//if there are 4 vertices  in the contour(It should be a quadrilateral)
		else if(result->total==4 )
		{
			//iterating through each point
			CvPoint *pt[4];
			for(int i=0;i<4;i++){
				pt[i] = (CvPoint*)cvGetSeqElem(result, i);
			}

			//drawing lines around the quadrilateral
			cvLine(img, *pt[0], *pt[1], cvScalar(0,255,0),4);
			cvLine(img, *pt[1], *pt[2], cvScalar(0,255,0),4);
			cvLine(img, *pt[2], *pt[3], cvScalar(0,255,0),4);
			cvLine(img, *pt[3], *pt[0], cvScalar(0,255,0),4);
		}

		//if there are 7 vertices  in the contour(It should be a heptagon)
		else if(result->total ==7  )
		{
			//iterating through each point
			CvPoint *pt[7];
			for(int i=0;i<7;i++){
				pt[i] = (CvPoint*)cvGetSeqElem(result, i);
			}

			//drawing lines around the heptagon
			cvLine(img, *pt[0], *pt[1], cvScalar(0,0,255),4);
			cvLine(img, *pt[1], *pt[2], cvScalar(0,0,255),4);
			cvLine(img, *pt[2], *pt[3], cvScalar(0,0,255),4);
			cvLine(img, *pt[3], *pt[4], cvScalar(0,0,255),4);
			cvLine(img, *pt[4], *pt[5], cvScalar(0,0,255),4);
			cvLine(img, *pt[5], *pt[6], cvScalar(0,0,255),4);
			cvLine(img, *pt[6], *pt[0], cvScalar(0,0,255),4);
		}

		//obtain the next contour
		contour = contour->h_next;
	}

	//show the image in which identified shapes are marked
	cvNamedWindow("Tracked");
	cvShowImage("Tracked",img);

	cvWaitKey(0); //wait for a key press

	//cleaning up
	cvDestroyAllWindows();
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&img);
	cvReleaseImage(&imgGrayScale);

	return 0;
}
