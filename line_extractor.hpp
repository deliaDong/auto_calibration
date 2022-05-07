#ifndef line_extractor_hpp
#define line_extractor_hpp

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/ximgproc/fast_line_detector.hpp>
//#include <opencv2/ximgproc/edge_drawing.hpp>
#include <iostream>
#include <stdio.h>
#include "glog/logging.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.h"

using namespace cv;
using namespace std;

/* Struct holding the parameters for one camera */
struct CalibParams {
    // Eigen::Matrix3d intrinsic;
    // Eigen::Vector4d distortion;
    // Sophus::SE3 extrinsic;
    double pitch;
    double yaw;
    double roll;
    double dX;
    double dY;
    double cX;
    double cY;
    double d1;
    double d2;
    double d3;
    double d4;
};


/* Struct for all four cameras */
struct camera_set {
    CalibParams camera_F;
    CalibParams camera_L;
    CalibParams camera_B;
    CalibParams camera_R;
};


/* Struct for lanemarks */
struct lanemarks {

};

    
struct one_frame_lines {

};

struct one_frame_lines_set {

};

struct vanishing_pts {

};


/* create a new CalibParams struct and initialize it
 * Parameters see definition for CalibParams
 */
CalibParams* camera_new;

one_frame_lines_set* one_frame_set_new();

/* Initialize the intrinsic parameters based on a camera
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * cam_matrix: a pointer holding the return value of the camera matrix
 * dist_coeffs: a pointer holding the return value of the distortion coefficients
 */
void init_intrinsic(CalibParams* camera, Eigen::Matrix3d cam_matrix, Eigen::Vector4d dist_coeffs);


/* Calculate the rotation matrix R based on a camera's pitch, yaw, and roll angles
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * R: a pointer holding the return value of the rotation matrix
 */
void rotation_homography(CalibParams* camera, Eigen::Matrix3d &R);


/* Calculate the translation matrix R based on the rotation matrix
 * ******rotation_homography MUST BE CALLED FIRST******
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * R: a pointer holding the return value of the rotation matrix
 * T: a pointer holding the return value of the translation matrix
 */
void translation_homography(CalibParams* camera, Eigen::Matrix3d &R, Sophus::SE3 &T);

void ENURotationFromEuler(Eigen::Matrix3d &R);

/* Adjust the orientation of the rotation and translation matrix from zyx to yxz
 *
 * R: a pointer to the rotation matrix
 * T: a pointer to the translation matrix
 */
void homography_adjust(Eigen::Matrix3d &R, Sophus::SE3 &T);


//RANSAC fit 2D straight line
//Input parameters: points--input point set
// iterations--number of iterations
// sigma--The acceptable difference between the data and the model, the lane line pixel bandwidth is generally about 10
//              （Parameter use to compute the fitting score）
// k_min/k_max--The value range of the slope of the fitted straight line.
// Considering that the slope of the left and right lane lines in the image is within a certain range,
// Adding this parameter can also avoid detecting vertical and horizontal lines
//Output parameters: line--fitted line parameters, It is a vector of 4 floats
//              (vx, vy, x0, y0) where (vx, vy) is a normalized
//              vector collinear to the line and (x0, y0) is some
//              point on the line.
//Return value: none
//void fitLineRansac(const std::vector<cv::Point2f>& points,
//                   cv::Vec4f &line,
//                   int iterations = 1000,
//                   double sigma = 1.,
//                   double k_min = -7.,
//                   double k_max = 7.);


/* Detects the lanemark lines in the birdeye image
 *
 * birdeye_img: a pointer to the birdeye image
 */
void detect_line(cv::Mat birdeye_img);


/* transform one fisheye image into a birdeye image with the camera parameters
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * img: a pointer to the fisheye image
 */
cv::Mat&  birdeye_oneview_transform(CalibParams* camera, cv::Mat fisheye_img);


void oneview_extract_line(cv::Mat birdeye_img);

/* transform all four fisheye images with four cameras' parameters
 *
 * cameras: a pointer to camera_set struct holding parameters of all four cameras
 * all_img: a pointer to all four fisheye images
 */
vector<cv::Mat>& birdeye_transform(camera_set *cameras, vector<cv::Mat> all_img);

void line_pairing();

//RANSAC fit 2D straight line
//Input parameters: points--input point set
// iterations--number of iterations
// sigma--The acceptable difference between the data and the model, the lane line pixel bandwidth is generally about 10
//              （Parameter use to compute the fitting score）
// k_min/k_max--The value range of the slope of the fitted straight line.
// Considering that the slope of the left and right lane lines in the image is within a certain range,
// Adding this parameter can also avoid detecting vertical and horizontal lines
//Output parameters: line--fitted line parameters, It is a vector of 4 floats
//              (vx, vy, x0, y0) where (vx, vy) is a normalized
//              vector collinear to the line and (x0, y0) is some
//              point on the line.
//Return value: none
void fitLineRansac();

static float GetDist();

// get angle with positive vale between 2 line segments
static float GetAngle();

// get angle with positive value between input line and iamge x axis
static float GetAngle();

static float GetX();

// determine whether 2 lines is colinear
static float GetSimilarity();

static void Kmeans();

void FilterLines();

void duplicate_filter();

void length_filter();

void range_filter();

void orientation_filter();

void average_vanish_pt();

void get_vanish_pt();

double get_pitch();

void get_Hmax();

void side_cam_Hmax();

void front_back_cam_Hmax();

void get_h1();

void get_h2();

void get_h3();

void calculate_H();

void get_virtual_K();

void get_updated_R();

double get_rotation_axis_and_angle();

void outlier_filter();

float get_slope();

float get_intercept();

float get_x();

bool location_check();

void find_closest_lane();

bool cluter_filter();

void get_cluster_median();

void cluster_vpt_set();

void lanes_filter();

#endif /* line_extractor_hpp */

