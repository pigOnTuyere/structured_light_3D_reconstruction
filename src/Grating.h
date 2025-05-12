#pragma once
#define _USE_MATH_DEFINES // ��ĳЩ����������Visual Studio������Ҫ����������M_PI�Ķ���
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <math.h>
#include <cmath>

#define m_DebugMode 0

using namespace std;
using namespace cv;

class Grating
{
public:
	Grating();
	~Grating();
public:
	const int grating_rows = 1200, grating_cols = 1600;

	cv::Mat m_PhiPhaseList[3] =
	{
		cv::Mat::zeros(grating_rows, grating_cols,CV_64FC1),
		cv::Mat::zeros(grating_rows, grating_cols,CV_64FC1),
		cv::Mat::zeros(grating_rows, grating_cols,CV_64FC1)
	};//������λ0~2pi

	cv::Mat m_AbsolutePhiPhase = cv::Mat::zeros(grating_rows, grating_cols, CV_64FC1);//������λ
	cv::Mat m_ShadowflagImage = cv::Mat::zeros(grating_rows, grating_cols, CV_8UC1);

	cv::Mat FAI_view;//������λ


private:
	//const double T1 = 140, T2 = 134, T3 = 129;
	const double T1 = 140, T2 = 134, T3 = 129;
	//const double T1 = 70, T2 = 64, T3 = 59;


	//const double T1 = 70, T2 = 64, T3 = 59;
	//const double T1 = 1, T2 = 1, T3 = 1;
	const double p1 = 1.0 / T1, p2 = 1.0 / T2, p3 = 1.0 / T3;
	const double p12 = p1 * p2 / (p2 - p1), p23 = p2 * p3 / (p3 - p2);
	const double T12 = 1.0 / p12, T23 = 1.0 / p23;
	const double p123 = p12 * p23 / (p23 - p12);
	const double T123 = 1.0 / p123;

public:

	void removeShadow(vector<cv::Mat> grating_img);/*���ƶȷָ�ȥ����Ӱ,��ͼƬ��ֵ����
	������ά�ؽ��е��ж�����(�ڶ�Ƶ��������û���õ�)*/

	//��cornerpoints��Ӧ��theta����
	std::vector<std::vector<double>> samples_FAI;
	void N_StepPhaseShifting(vector<cv::Mat> grating_img);//�Ĳ����Ʒ�
	void ThreeFrequencyUnwrap();

	//��cornerpoints��Ӧ��theta����
	std::vector<std::vector<double>>
		threeFrequencyHeterodyneImproved(std::vector<cv::Point2f> cornerpoints_temp, vector<cv::Mat> grating_img);


};