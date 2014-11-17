/*
*  drones-267, automated landing using pose estimation
*  Copyright (C) {2013}  {Constantin Berzan, Nahush Bhanage, Sunil Shah}
*  
*  https://github.com/ssk2/drones-267
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
* 
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* 
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "geometry.h"
#include "math.h"

#include <cstdio>
using namespace std; // DEBUG


Mat_<double> estimateRotTransl(
    Mat_<double> const worldPts,
    Mat_<double> const imagePts)
{
    assert(imagePts.cols == 3);
    assert(worldPts.cols == 3);
    assert(imagePts.rows == worldPts.rows);
    // TODO verify all worldPts have z=0

    int n = imagePts.rows;
    Mat_<double> worldPtsValid;
    Mat_<double> imagePtsValid;
    for (int i = 0; i < n; i++) {
    	if (imagePts(i,2) != 0) {
    		worldPtsValid.push_back(worldPts.row(i));
    		imagePtsValid.push_back(imagePts.row(i));
    	}
    }

    // See "pose estimation" section in the paper.
    n = imagePtsValid.rows;

    // check that there are enough number of points (>=4).
    if (n < 5) {
    	cout << "No enough Points" << endl;
    	Mat_<double> rotTransl(3, 4);
    	rotTransl << 0,0,0,0,
    			     0,0,0,0,
    			     0,0,0,0;
    	return rotTransl;
    }
    // Set up linear system of equations.
    Mat_<double> F(2 * n, 9);
    for(int i = 0; i < n; i++)
    {
        F(2 * i, 0) = worldPtsValid(i, 0);
        F(2 * i, 1) = 0;
        F(2 * i, 2) = -worldPtsValid(i, 0) * imagePtsValid(i, 0);
        F(2 * i, 3) = worldPtsValid(i, 1);
        F(2 * i, 4) = 0;
        F(2 * i, 5) = -worldPtsValid(i, 1) * imagePtsValid(i, 0);
        F(2 * i, 6) = 1;
        F(2 * i, 7) = 0;
        F(2 * i, 8) = -imagePtsValid(i, 0);

        F(2 * i + 1, 0) = 0;
        F(2 * i + 1, 1) = worldPtsValid(i, 0);
        F(2 * i + 1, 2) = -worldPtsValid(i, 0) * imagePtsValid(i, 1);
        F(2 * i + 1, 3) = 0;
        F(2 * i + 1, 4) = worldPtsValid(i, 1);
        F(2 * i + 1, 5) = -worldPtsValid(i, 1) * imagePtsValid(i, 1);
        F(2 * i + 1, 6) = 0;
        F(2 * i + 1, 7) = 1;
        F(2 * i + 1, 8) = -imagePtsValid(i, 1);
    }

    // Find least-squares estimate of rotation + translation.
    SVD svd(F);

    Mat_<double> rrp = svd.vt.row(8);
    rrp = rrp.clone().reshape(0, 3).t();
    if(rrp(2, 2) < 0) {
        rrp *= -1;  // make sure depth is positive
    }
    // cout << "rrp: " << rrp << endl;

    Mat_<double> transl = \
        2 * rrp.col(2) / (norm(rrp.col(0)) + norm(rrp.col(1)));
    // cout << "transl: " << transl << endl;

    Mat_<double> rot = Mat_<double>::zeros(3, 3);
    rrp.col(0).copyTo(rot.col(0));
    rrp.col(1).copyTo(rot.col(1));
    SVD svd2(rot);
    rot = svd2.u * svd2.vt;
    if(determinant(rot) < 0) {
        rot.col(2) *= -1; // make sure it's a valid rotation matrix
    }
    if(abs(determinant(rot) - 1) > 1e-10) {
        cerr << "Warning: rotation matrix has determinant " \
             << determinant(rot) << " where expected 1." << endl;
    }
    // cout << "rot: " << rot << endl;

    Mat_<double> rotTransl(3, 4);
    rot.col(0).copyTo(rotTransl.col(0));
    rot.col(1).copyTo(rotTransl.col(1));
    rot.col(2).copyTo(rotTransl.col(2));
    transl.copyTo(rotTransl.col(3));

//    cout << "rotTransl = " << endl << rotTransl << endl << endl;

    return rotTransl;
}

Mat_<double> estimatePose_SVD(Mat_<double> const imagePts, Mat_<double> const worldPts)
{
    // Note that given R, t, the camera location in world
    // coordinates is not just t, but instead -inv(R)*t.

    Mat_<double> const rotTransl = estimateRotTransl(worldPts, imagePts);
    Mat_<double> rotMatrix(3, 3);
    Mat_<double> translation(3, 1);
    rotTransl.col(0).copyTo(rotMatrix.col(0));
    rotTransl.col(1).copyTo(rotMatrix.col(1));
    rotTransl.col(2).copyTo(rotMatrix.col(2));
    rotTransl.col(3).copyTo(translation);
    Mat_<double> cameraLoc = -rotMatrix.t() * translation;
    Mat_<double> simplePose(4, 1);
    simplePose(0) = cameraLoc(0);  // x
    simplePose(1) = cameraLoc(1);  // y
    simplePose(2) = cameraLoc(2);  // z
    // Yaw (rotation around z axis):
    // See http://planning.cs.uiuc.edu/node103.html
    simplePose(3) = atan2(rotMatrix(1, 0), rotMatrix(0, 0));
    // cout << "simplePose: " << simplePose << endl;
    return simplePose;
}

Mat_<double> estimatePose_GEO(Mat_<double> const imagePts, Mat_<double> const worldPts)
{
	// validation check.
    assert(imagePts.cols == 3);
    assert(worldPts.cols == 3);
    assert(imagePts.rows == worldPts.rows);
    // TODO verify all worldPts have z=0

    // extract valid points only.
    int n = imagePts.rows;
    Mat_<double> worldPtsValid;
    Mat_<double> imagePtsValid;
    for (int i = 0; i < n; i++) {
    	if (imagePts(i,2) != 0) {
    		worldPtsValid.push_back(worldPts.row(i));
    		imagePtsValid.push_back(imagePts.row(i));
    	}
    }

    // check that there are enough number of points (>=4).
	n = imagePtsValid.rows;
	if (n < 3) {
		cout << "No enough Points" << endl;
	    Mat_<double> simplePose(4, 1);
	    simplePose = Mat::zeros(4, 1, CV_64F);;
	    return simplePose;
	}

    /*
     * calculate height
     */
	Mat_<double> worldPtsMag(n,1), imagePtsMag(n,1);
	magnitude( worldPtsValid.col(0).rowRange(1,n) - worldPtsValid.col(0).row(0),
			   worldPtsValid.col(1).rowRange(1,n) - worldPtsValid.col(1).row(0),
			   worldPtsMag) ;
	magnitude( imagePtsValid.col(0).rowRange(1,n) - imagePtsValid.col(0).row(0),
			   imagePtsValid.col(1).rowRange(1,n) - imagePtsValid.col(1).row(0),
			   imagePtsMag);
	Mat_<double> heightArray = worldPtsMag/imagePtsMag;
	double height = mean(heightArray).val[0];

	/*
	 * calculate camera position in transformed coordinates
	 */
	// define landing pad coordinate basis {x_h, y_h}.
	Vec2f x_h = normalize(Vec2f(imagePtsValid.at<double>(1,0) - imagePtsValid.at<double>(0,0),
				      imagePtsValid.at<double>(1,1) - imagePtsValid.at<double>(0,1)));
	Vec2f y_h = Vec2f(-x_h.val[1], x_h.val[0]);

	Vec2f camCenter = -Vec2f(imagePtsValid.at<double>(0,0), imagePtsValid.at<double>(0,1));
	camCenter = Vec2f(x_h.dot(camCenter), y_h.dot(camCenter)) * (worldPtsValid.at<double>(1,0) - worldPtsValid.at<double>(0,0))
			                                                  / (imagePtsValid.at<double>(1,0) - imagePtsValid.at<double>(0,0));

	// calculate yaw angle.
	double yaw = atan2(x_h.val[1], x_h.val[0]);

	/* assemble pose estimate */
    Mat_<double> simplePose(4, 1);
    simplePose(0) = camCenter.val[0];  	// x
    simplePose(1) = camCenter.val[1];  	// y
    simplePose(2) = height;  			// z
    simplePose(3) = yaw;				// yaw

    return simplePose;
}

Mat_<double> getWorldPts()
{
    Mat_<double> worldPtsHom(23, 3, CV_64FC1);
    for(int i = 0; i < worldPtsHom.rows; i++) {
        worldPtsHom[i][2] = 0;  // z = 0
    }

    /* Define coordinate in millimeter */
    double L_centroid = 300.0;
    // Large Circle:
    worldPtsHom(0, 0) = 0;
    worldPtsHom(0, 1) = 0;

    // Small Circle:
    worldPtsHom(1, 0) = 150;
    worldPtsHom(1, 1) = 0;

    // Square:
    double L_square = 80.0;
    worldPtsHom(2, 0) = L_centroid;		// centroid
    worldPtsHom(2, 1) = 0;
    worldPtsHom(3, 0) = L_centroid+L_square;		// upper right
    worldPtsHom(3, 1) = 0+L_square;
    worldPtsHom(4, 0) = L_centroid+L_square;		// lower right
    worldPtsHom(4, 1) = 0-L_square;
    worldPtsHom(5, 0) = L_centroid-L_square;		// lower left
    worldPtsHom(5, 1) = 0+L_square;
    worldPtsHom(6, 0) = L_centroid-L_square;		// upper left
    worldPtsHom(6, 1) = 0-L_square;

    // Rectangle:
    double L_rect = 55.0;
    double H_rect = 190.0;
    worldPtsHom(7, 0) = 0;							// centroid
    worldPtsHom(7, 1) = L_centroid;
    worldPtsHom(8, 0) = 0+L_rect/2;					// upper right
    worldPtsHom(8, 1) = L_centroid+H_rect/2;
    worldPtsHom(9, 0) = 0+L_rect/2;					// lower right
    worldPtsHom(9, 1) = L_centroid-H_rect/2;
    worldPtsHom(10, 0) = 0-L_rect/2;				// lower left
    worldPtsHom(10, 1) = L_centroid-H_rect/2;
    worldPtsHom(11, 0) = 0-L_rect/2;				// upper left
    worldPtsHom(11, 1) = L_centroid+H_rect/2;

    // Triangle:
    double L_tri = 175.0;
    worldPtsHom(12, 0) = -L_centroid;					// centroid
    worldPtsHom(12, 1) = 0;
    worldPtsHom(13, 0) = -L_centroid+L_tri*sqrt(3)/6;	// upper right
    worldPtsHom(13, 1) = 0+L_tri/2;
    worldPtsHom(14, 0) = -L_centroid+L_tri*sqrt(3)/6;	// lower right
    worldPtsHom(14, 1) = 0-L_tri/2;
    worldPtsHom(15, 0) = -L_centroid-L_tri*sqrt(3)/3;	// left
    worldPtsHom(15, 1) = 0;

    // Hexagon:
    double L_hex = 100.0;
    worldPtsHom(16, 0) = 0;
    worldPtsHom(16, 1) = -L_centroid;
    worldPtsHom(17, 0) = L_hex/2;
    worldPtsHom(17, 1) = -L_centroid+L_hex*sqrt(3)/2;
    worldPtsHom(18, 0) = L_hex;
    worldPtsHom(18, 1) = -L_centroid;
    worldPtsHom(19, 0) = L_hex/2;
    worldPtsHom(19, 1) = -L_centroid-L_hex*sqrt(3)/2;
    worldPtsHom(20, 0) = -L_hex/2;
    worldPtsHom(20, 1) = -L_centroid-L_hex*sqrt(3)/2;
    worldPtsHom(21, 0) = -L_hex;
    worldPtsHom(21, 1) = -L_centroid;
    worldPtsHom(22, 0) = -L_hex/2;
    worldPtsHom(22, 1) = -L_centroid+L_hex*sqrt(3)/2;

    worldPtsHom = worldPtsHom * SCALE_FACTOR;

//    cout << worldPtsHom << endl;
    return worldPtsHom;
}

Mat_<double> cartToHom(Mat_<double> const cart)
{
    Mat_<double> hom(cart.rows, cart.cols + 1, 1.0);
    Mat_<double> tmp = hom(Range::all(), Range(0, cart.cols));
    cart.copyTo(tmp);
    return hom;
}

Mat_<double> homToCart(Mat_<double> const hom)
{
    // Don't know how to do this efficiently in OpenCV.
    Mat_<double> cart(hom.rows, hom.cols - 1);
    for(int r = 0; r < hom.rows; r++) {
        for(int c = 0; c < hom.cols - 1; c++) {
            cart(r, c) = hom(r, c) / hom(r, hom.cols - 1);
        }
    }
    return cart;
}

Mat_<double> rotAxisAngleToRotMatrix(
    Mat_<double> const rotAxis,
    double rotAngle)
{
    assert(rotAxis.rows == 3);
    assert(rotAxis.cols == 1);
    assert(abs(norm(rotAxis) - 1) < 1e-10);

    // Skew-symmetric matrix representing cross product by s:
    Mat_<double> sHat(3, 3);
    sHat << 0, -rotAxis(2), rotAxis(1), \
            rotAxis(2), 0, -rotAxis(0), \
            -rotAxis(1), rotAxis(0), 0;

    // Roderiques formula:
    Mat_<double> rotation = Mat_<double>::eye(3,3) \
        + sin(rotAngle) * sHat \
        + (1 - cos(rotAngle)) * sHat * sHat;
    return rotation;
}

Mat_<double> worldHomToCameraHom(
    Mat_<double> const worldPtsHom,
    Mat_<double> const rotMatrix,
    Mat_<double> const translation)
{
    assert(worldPtsHom.cols == 4);
    assert(rotMatrix.rows == 3);
    assert(rotMatrix.cols == 3);
    assert(translation.rows == 3);
    assert(translation.cols == 1);

    // Convert rotMatrix + translation into a linear transformation in
    // homogeneous coordinates.
    Mat_<double> rigidMotion = Mat_<double>::zeros(4, 4);
    rotMatrix.copyTo(rigidMotion(Range(0, 3), Range(0, 3)));
    translation.copyTo(rigidMotion(Range(0, 3), Range(3, 4)));
    rigidMotion(3, 3) = 1;
    // cout << "rigidMotion: " << rigidMotion << endl;

    // Assuming camera calibration matrix is identity.
    // Note that OpenCV treats size as "[cols rows]", so matrix multiplication
    // has to be done backwards (transpose everything).
    Mat_<double> projection = Mat_<double>::eye(3, 4);
    Mat result = projection * rigidMotion * worldPtsHom.t();
    return result.t();
}

void drawImagePts(Mat image, Mat_<double> const imagePts)
{
    assert(imagePts.rows == 24);
    assert(imagePts.cols == 2);
    double minVal, maxVal;
    minMaxIdx(imagePts, &minVal, &maxVal);
    double scale = 1.0 * image.rows / (maxVal - minVal);
    Point points[6][4];
    for(int i = 0; i < 24; i++) {
        points[i / 4][i % 4] = Point(
            scale * (imagePts(i, 0) - minVal),
            scale * (imagePts(i, 1) - minVal));
    }
    Point const* points2[] = {
        points[0], points[1], points[2],
        points[3], points[4], points[5]
    };  // stupid fillPoly won't acccept points directly
    int numPoints[] = { 4, 4, 4, 4, 4, 4 };
    fillPoly(image, points2, numPoints, 6, Scalar(255, 255, 255));

    // LEFT TODO: draw a rectangle around everything, otherwise it's hard to
    // tell what's going on...
}

Mat_<double> getCameraMatrix()
{
    Mat_<double> cameraMatrix(3, 3);
    cameraMatrix << 682,   0, 320,
                      0, 682, 240,
                      0,   0,   1;
    return cameraMatrix;
}

Mat_<double> calibrateImagePoints(Mat_<double> const imagePts)
{
    Mat_<double> const cameraMatrix = getCameraMatrix();
    assert(imagePts.cols == 3);
    Mat_<double> calibratedImagePts = Mat::zeros(imagePts.rows, 3, CV_64F);
    for(int i = 0; i < imagePts.rows; i++) {
        calibratedImagePts(i, 0) = \
            (imagePts(i, 0) - cameraMatrix(0, 2)) / cameraMatrix(0, 0);
        calibratedImagePts(i, 1) = \
            -(imagePts(i, 1) - cameraMatrix(1, 2)) / cameraMatrix(1, 1);
        calibratedImagePts(i, 2) = imagePts(i, 2);
    }
    return calibratedImagePts;
}

Mat_<double> unCalibrateImagePoints(Mat_<double> const calibratedImagePts)
{
    Mat_<double> const cameraMatrix = getCameraMatrix();
    assert(calibratedImagePts.cols == 2);
    Mat_<double> imagePts(calibratedImagePts.rows, 2);
    for(int i = 0; i < imagePts.rows; i++) {
        imagePts(i, 0) = \
            calibratedImagePts(i, 0) * cameraMatrix(0, 0) + cameraMatrix(0, 2);
        imagePts(i, 1) = \
            calibratedImagePts(i, 1) * cameraMatrix(1, 1) + cameraMatrix(1, 2);
    }
    return imagePts;
}

Mat_<double> CoordTransform ( double yawAngle, double pitchAngle, double rollAngle ) {

	Mat_<double> Axes   = Mat::eye(3,3,CV_32FC1);
	Mat_<double> yawAxis = Axes.col(2);
	Mat_<double> yawMatrix   = rotAxisAngleToRotMatrix(yawAxis, yawAngle);

	Axes = yawMatrix * Axes;
	Mat_<double> pitchAxis = Axes.col(1);
	Mat_<double> pitchMatrix   = rotAxisAngleToRotMatrix(pitchAxis, pitchAngle);

	Axes = pitchMatrix * Axes;
	Mat_<double> rollAxis = Axes.col(0);
	Mat_<double> rollMatrix   = rotAxisAngleToRotMatrix(rollAxis, rollAngle);

	Mat_<double>rotMatrix = rollMatrix * pitchMatrix * yawMatrix;

	Mat_<double>transformMatrix = -Mat::eye(4,4,CV_32FC1);
	Mat_<double>temp = transformMatrix (cv::Rect(0,0,3,3));
	rotMatrix.copyTo(temp);

//	std::cout << "transformMatrix = " << std::endl;
//	std::cout << transformMatrix << std::endl;

	return transformMatrix;
}
