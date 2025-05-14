#pragma once
#define _USE_MATH_DEFINES // Required in some compilers (e.g., Visual Studio) to enable M_PI definition
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
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
		: image_rows(params.H), image_cols(params.W), T1(params.T1), T2(params.T2), T3(params.T3)
	{};
	~Phaser() = default;
public:
	const int image_rows = 1200, image_cols = 1600;
	const double T1, T2, T3;

	cv::Mat m_phase_List[3] =
	{
		cv::Mat::zeros(image_rows, image_cols, CV_64FC1),
		cv::Mat::zeros(image_rows, image_cols, CV_64FC1),
		cv::Mat::zeros(image_rows, image_cols, CV_64FC1)
	}; // Wrapped phase from 0 to 2pi for each frequency

	cv::Mat m_absolute_phase = cv::Mat::zeros(image_rows, image_cols, CV_64FC1); // Absolute phase
	cv::Mat m_ShadowflagImage = cv::Mat::zeros(image_rows, image_cols, CV_8UC1); // Shadow mask

	cv::Mat FAI_view; // Absolute phase visualization

public:

	// Modulation-based shadow removal. Binarizes image for use as a mask in 3D reconstruction. 
	// (Not used in heterodyne unwrapping.)
	void removeShadow(const std::vector<cv::Mat>& phaser_img);

	// Compute theta samples corresponding to cornerpoints
	std::vector<std::vector<double>> samples_FAI;

	// Four-step phase shifting method
	void N_StepPhaseShifting(const vector<cv::Mat>& phaser_img);

	// Three-frequency phase unwrapping
	void ThreeFrequencyUnwrap();

	// Compute theta samples corresponding to cornerpoints using improved heterodyne method
	std::vector<std::vector<double>>
		threeFrequencyHeterodyneImproved(std::vector<cv::Point2f> cornerpoints_temp, vector<cv::Mat> phaser_img);
};
