#pragma once
#include<iostream>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>

#include<Eigen/Dense>
#include<Eigen/SVD>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>
#include<math.h>
#include<string>
#include<fstream>
#include"Phaser.h"
#include"Config.h"

class CameraDlp
{
public:
	CameraDlp(const Config& params)
		: image_rows(params.H), image_cols(params.W), output_dir(params.output_dir), center_size(params.center_size)
	{};
	~CameraDlp() = default;

public:
	int image_rows, image_cols;						 // Image resolution from the camera
	std::string output_dir;							// Output directory path
	double center_size;								//The distance between the center of the calibration plate

public:
	std::vector<std::vector<cv::Point2f>> corner_points;   // Corner points in image coordinates (from calibration board)
	std::vector<std::vector<cv::Point2f>> center_points;
	std::vector<std::vector<cv::Point3f>> world_points;    // Corresponding world coordinates of the calibration board

	cv::Mat camera_matrix_cv;						// Camera intrinsic matrix (OpenCV format)
	Eigen::Matrix<double, 3, 3> camera_matrix_eigen;       // Camera intrinsic matrix (Eigen format)
	cv::Mat dist_coefficients;							// Distortion coefficients: k1, k2, p1, p2, k3
	std::vector<cv::Mat> rotation_vectors;				// Rotation vectors: from chessboard to camera coordinate system
	std::vector<cv::Mat> translation_vectors;			 // Translation vectors
	std::vector<cv::Mat> H;							// Homography matrices
	std::vector<Eigen::Matrix3d> G;
	double pixel_m = 0.0;							// Real-world length of one pixel (in meters)
	std::vector<std::vector<cv::Point3f>> camera_points;   // 3D points in the camera coordinate system

	const cv::Size circleboard_size = cv::Size(11, 9);		// Size of the circular calibration board (cols, rows)

	std::vector<cv::Point2f> corner_points_temp;

	Eigen::MatrixXf getCameraParamFile();				 // Load camera parameter file
	Eigen::MatrixXf geta1_a8File();					 // Load a1-a8 parameter file

	// Detect circular grid centers in the calibration image
	bool detectCircleGridPoints(cv::Mat calib_img);

	// Perform camera calibration using the chessboard
	void cameraCalibrationChessBoard();

	// Compute the coordinates of corner points in the camera coordinate system
	void calculateCameraPoints();

	// Map phase to 3D coordinates using 8 parameters
	void phase3DPointsMapping(std::vector<std::vector<double>> samples_FAI);

	// Calculate 3D point cloud
	void calculate3DPoints(cv::Mat FAI_phase, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, cv::Mat shadowflag_img);

	// Calculate 3D feature points
	void calculate3D_FeaturePoint(cv::Mat FAI_phase, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, cv::Mat shadowflag_img);

	// Calculate 3D points with texture mapping
	void calculate3DPoints_TextureMapping(cv::Mat FAI_phase, cv::Mat source_picture, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_ptr, cv::Mat shadowflag_img);
};
