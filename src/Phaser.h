#pragma once
#define _USE_MATH_DEFINES // ��ĳЩ����������Visual Studio������Ҫ����������M_PI�Ķ���
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <math.h>
#include <cmath>
#include "Config.h"

#define m_DebugMode 0

using namespace std;
using namespace cv;

class Phaser
{
public:
	Phaser(const Config& params)
		:image_rows(params.H), image_cols(params.W),T1(params.T1),T2(params.T2),T3(params.T3)
	{};
	~Phaser()=default;
public:
	const int image_rows = 1200, image_cols = 1600;

	cv::Mat m_phase_List[3] =
	{
		cv::Mat::zeros(image_rows, image_cols,CV_64FC1),
		cv::Mat::zeros(image_rows, image_cols,CV_64FC1),
		cv::Mat::zeros(image_rows, image_cols,CV_64FC1)
	};//������λ0~2pi

	cv::Mat m_absolute_phase = cv::Mat::zeros(image_rows, image_cols, CV_64FC1);//������λ
	cv::Mat m_ShadowflagImage = cv::Mat::zeros(image_rows, image_cols, CV_8UC1);

	cv::Mat FAI_view;//������λ


public:
	const double T1 , T2 , T3 ;

	//const double p1 = 1.0 / T1, p2 = 1.0 / T2, p3 = 1.0 / T3;
	//const double p12 = p1 * p2 / (p2 - p1), p23 = p2 * p3 / (p3 - p2);
	//const double T12 = 1.0 / p12, T23 = 1.0 / p23;
	//const double p123 = p12 * p23 / (p23 - p12);
	//const double T123 = 1.0 / p123;

public:


	void removeShadow(const std::vector<cv::Mat>& phaser_img);/*���ƶȷָ�ȥ����Ӱ,��ͼƬ��ֵ����
	������ά�ؽ��е��ж�����(�ڶ�Ƶ��������û���õ�)*/

	//��cornerpoints��Ӧ��theta����
	std::vector<std::vector<double>> samples_FAI;
	void N_StepPhaseShifting(const vector<cv::Mat>& phaser_img);//�Ĳ����Ʒ�
	void ThreeFrequencyUnwrap();

	//��cornerpoints��Ӧ��theta����
	std::vector<std::vector<double>>
		threeFrequencyHeterodyneImproved(std::vector<cv::Point2f> cornerpoints_temp, vector<cv::Mat> phaser_img);


};