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

#include <opencv2/opencv.hpp>
using namespace cv;

/**
 * Estimate rotation and translation of camera w.r.t. world.
 *
 * Takes a point correspondence between worldPts and imagePts. Computes the
 * rotation and translation of the camera coordinate frame w.r.t. the world
 * coordinate frame. The worldPts are expected to have z=0.
 *
 * Inputs:
 * worldPts (Nx3) = world points
 * imagePts (Nx2) = image points
 *
 * Returns:
 * pose (3x4) = [3x3] rotation and [3x1] translation of camera w.r.t. world
 */
Mat_<double> estimateRotTransl(
    Mat_<double> const worldPts,
    Mat_<double> const imagePts);

/**
 * Estimate camera pose w.r.t. hardcoded landing pad.
 *
 * Inputs:
 * imagePts (Nx2) = image points in the order described in the paper
 *
 * Returns:
 * pose (4x1) = x, y, z, yaw coords of camera
 */
Mat_<double> estimatePose(Mat_<double> const imagePts);

/**
 * Return landing pad points in the order described in the paper.
 *
 * The coordinates are in millimeters.
 * The origin of the coordinate system is point #3, the pad center.
 *
 * Returns:
 * worldPts (24x3) = world points
 */
Mat_<double> getWorldPts();

/**
 * Convert from Cartesian to homogeneous coordinates.
 *
 * Inputs:
 * cart (NxD) = N points in D dimensions
 *
 * Returns:
 * hom (NxD+1) = N points in homogeneous coordinates.
 */
Mat_<double> cartToHom(Mat_<double> const cart);

/**
 * Convert from homogeneous to Cartesian coordinates.
 *
 * Inputs:
 * hom (NxD+1) = N points in homogeneous coordinates.
 *
 * Returns:
 * cart (NxD) = N points in D dimensions
 */
Mat_<double> homToCart(Mat_<double> const hom);

/**
 * Compute 3D rotation matrix given rotation vector and angle.
 *
 * Inputs:
 * rotAxis (3x1) = unit vector indicating rotation axis
 * rotAngle = amount of rotation in radians
 *
 * Returns:
 * rotation (3x3) = rotation matrix
 */
Mat_<double> rotAxisAngleToRotMatrix(
    Mat_<double> const rotAxis,
    double rotAngle);

/**
 * Project points in the world to points in the image.
 *
 * Everything is in homogeneous coordinates.
 * We assume the camera calibration matrix is identity.
 *
 * Inputs:
 * worldPtsHom (Nx4) = world points in homogeneous coordinates
 * rotMatrix (3x3) = rotation of camera w.r.t. world
 * translation (3x1) = translation of camera w.r.t. world
 *
 * Returns:
 * imagePtsHom (Nx3) = image points in homogeneous coordinates
 */
Mat_<double> worldHomToCameraHom(
    Mat_<double> const worldPtsHom,
    Mat_<double> const rotMatrix,
    Mat_<double> const translation);

/**
 * Draw the landing pad given by imagePts onto the given image.
 *
 * Inputs:
 * imagePts (Nx2) = image points in the order described in the paper
 *
 * Outputs:
 * (modifies image)
 */
void drawImagePts(Mat image, Mat_<double> const imagePts);

/**
 * Return camera matrix.
 *
 * This function encapsulates the calibration info for our camera.
 *
 * Returns:
 * cameraMatrix (3x3)
 */
Mat_<double> getCameraMatrix();

/**
 * Convert image points to calibrated image points.
 *
 * This function encapsulates the calibration info for our camera.
 *
 * Inputs:
 * imagePts (Nx2) = uncalibrated image points
 *
 * Outputs:
 * calibratedImagePts (Nx2) = calibrated image points
 */
Mat_<double> calibrateImagePoints(Mat_<double> const imagePts);

/**
 * Invert the transformation done by `calibrateImagePoints`.
 *
 * Inputs:
 * calibratedImagePts (Nx2) = calibrated image points
 *
 * Outputs:
 * imagePts (Nx2) = uncalibrated image points
 */
Mat_<double> unCalibrateImagePoints(Mat_<double> const calibratedImagePts);

Mat_<double> CoordTransform (double yawAngle, double pitchAngle, double rollAngle);
