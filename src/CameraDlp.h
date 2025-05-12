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
#include"Grating.h"
#include"Config.h"


class CameraDlp
{

public:
	CameraDlp(const Config& params) :image_rows(params.H),
		image_cols(params.W),output_dir(params.output_dir) {
	};
	~CameraDlp() = default;
	int image_rows, image_cols;//���ͼƬ��С
	string output_dir; //

	std::vector<std::vector<cv::Point2f>> corner_points;//�궨��ͼ������
	std::vector<std::vector<cv::Point2f>> centerpoints;
	std::vector<std::vector<cv::Point3f>> world_points;//�궨����������
	cv::Mat camera_matrix_cv;//����ڲξ���
	Eigen::Matrix<double, 3, 3> camera_matrix_eigen;
	cv::Mat dist_coefficients;//k1 k2 p1 p2 k3
	std::vector< cv::Mat >  rotation_vectors;//��ת���������̸��������ϵ
	std::vector<cv::Mat>  translation_vectors;//ƽ������
	std::vector<cv::Mat> H;//��Ӧ����
	std::vector<Eigen::Matrix3d> G;
	double pixel_m = 0.0;//ÿ�����ر߳�ʵ�ʳ��� ��
	std::vector<std::vector<cv::Point3f>> camera_points;
	//cv::Mat shadow_img;//�׹����Ӱ��ͼƬ

	const cv::Size circleboard_size = cv::Size(11, 9);  //(7,7)  (6,7)
	double center_size = 20;//xmm  // 0.003  3  2  2.5


	std::vector<cv::Point2f > corner_points_temp;

	//const int w = 11; const int h = 8;
	Eigen::MatrixXf  getCameraParamFile();
	Eigen::MatrixXf geta1_a8File();

	//���Բ����������
	bool detectCircleGridPoints(cv::Mat calib_img);

	//�����������
	void cameraCalibrationChessBoard();

	//����ǵ����������ϵ������
	void calculateCameraPoints();

	//����8������
	void phase3DPointsMapping(std::vector<std::vector<double>> samples_FAI);


	void calculate3DPoints(cv::Mat FAI_phase, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, cv::Mat shadowflag_img);
	void calculate3D_FeaturePoint(cv::Mat FAI_phase, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, cv::Mat shadowflag_img);
	void calculate3DPoints_TextureMapping(cv::Mat FAI_phase, cv::Mat source_picture, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_ptr, cv::Mat shadowflag_img);


};
