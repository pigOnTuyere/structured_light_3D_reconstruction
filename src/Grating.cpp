#include "Grating.h"
#include <cmath>  // ���ͷ�ļ�

Grating::Grating() {}

Grating::~Grating() {}

void Grating::removeShadow(vector<cv::Mat> grating_img)  //���ƶȷָ�ȥ����Ӱ
{
	cv::Mat b_img[3] = {
		cv::Mat::zeros(grating_rows, grating_cols, CV_8UC1),
		cv::Mat::zeros(grating_rows, grating_cols, CV_8UC1),
		cv::Mat::zeros(grating_rows, grating_cols, CV_8UC1) };  //���ƶ�

	int thresh_shadow = 2;

	uchar* pb_img[3] = { NULL };

	int N = grating_img.size() / 3;

	cout << "N:" << N << endl;

	for (int y = 0; y < grating_rows; ++y) {
		std::vector<uchar*> p;

		for (int i = 0; i < grating_img.size(); ++i) {
			p.push_back(grating_img[i].ptr<uchar>(y));
		}

		pb_img[0] = b_img[0].ptr<uchar>(y);
		pb_img[1] = b_img[1].ptr<uchar>(y);
		pb_img[2] = b_img[2].ptr<uchar>(y);

		for (int x = 0; x < grating_cols; ++x) {
			int sinsum0 = 0;
			int cossum0 = 0;
			int sinsum1 = 0;
			int cossum1 = 0;
			int sinsum2 = 0;
			int cossum2 = 0;

			for (int k = 0; k < N; k++) {
				sinsum0 = sinsum0 + p[k][x] * sin(2 * k * M_PI / N);
				cossum0 = cossum0 + p[k][x] * cos(2 * k * M_PI / N);
				sinsum1 = sinsum1 + p[k + N][x] * sin(2 * k * M_PI / N);
				cossum1 = cossum1 + p[k + N][x] * cos(2 * k * M_PI / N);
				sinsum2 = sinsum2 + p[k + 2 * N][x] * sin(2 * k * M_PI / N);
				cossum2 = cossum2 + p[k + 2 * N][x] * cos(2 * k * M_PI / N);
			}

			double mm = 2.0 / N;

			pb_img[0][x] = mm * sqrt(pow(cossum0, 2) + pow(sinsum0, 2));
			pb_img[1][x] = mm * sqrt(pow(cossum1, 2) + pow(sinsum1, 2));
			pb_img[2][x] = mm * sqrt(pow(cossum2, 2) + pow(sinsum2, 2));

			// pb_img[0][x] = 0.5 * sqrt(pow(p[0][x] - p[2][x], 2) + pow(p[1][x] -
			// p[3][x], 2)); pb_img[1][x] = 0.5 * sqrt(pow(p[4][x] - p[6][x], 2) +
			// pow(p[5][x] - p[7][x], 2)); pb_img[2][x] = 0.5 * sqrt(pow(p[8][x] -
			// p[10][x], 2) + pow(p[9][x] - p[11][x], 2));

			if (pb_img[0][x] <= thresh_shadow || pb_img[1][x] <= thresh_shadow ||
				pb_img[2][x] <= thresh_shadow)
				m_ShadowflagImage.at<uchar>(y, x) = 0;
			else
				m_ShadowflagImage.at<uchar>(y, x) = 255;
		}
	}

	////���Ų�ͬƵ�ʵ���ֵͼ
	// cv::namedWindow("fai_img[0]", 0);
	// cv::resizeWindow("fai_img[0]", cv::Size(grating_cols / 2, grating_rows /
	// 2)); cv::imshow("fai_img[0]", m_ShadowflagImage); cv::waitKey(0);
}


void Grating::N_StepPhaseShifting(vector<cv::Mat> grating_img)
{
	const int N = grating_img.size() / 3;
	std::vector<double> sin_table(N), cos_table(N);
	for (int k = 0; k < N; ++k) {
		double angle = 2.0 * M_PI * k / N;
		sin_table[k] = sin(angle);
		cos_table[k] = cos(angle);
	}

	// ׼���������sin��cos�ۼӾ���������ΪCV_64FC1ȷ�����ȣ�
	cv::Mat sinsum[3], cossum[3];
	for (int i = 0; i < 3; ++i) {
		sinsum[i] = cv::Mat::zeros(grating_rows, grating_cols, CV_64FC1);
		cossum[i] = cv::Mat::zeros(grating_rows, grating_cols, CV_64FC1);
	}

	// ��ÿ��������ͼ���ۼӼ�Ȩ���
	for (int k = 0; k < N; ++k) {
		for (int g = 0; g < 3; ++g) {
			cv::Mat img_double;
			grating_img[k + g * N].convertTo(img_double, CV_64FC1); // ת��Ϊdouble�Ա����

			cv::Mat temp;
			cv::add(sinsum[g], img_double * sin_table[k], sinsum[g]);
			cv::add(cossum[g], img_double * cos_table[k], cossum[g]);
		}
	}

	// ������λͼ����һ��Ϊ uchar ͼ��
	cv::Mat fai_img[3];
	for (int g = 0; g < 3; ++g) {
		cv::Mat phase;
		///////////////////////////
		//phase = atan2(-sinsum[g], cossum[g]);
		cv::phase(cossum[g], -sinsum[g], phase); // atan2(-sin, cos)���Զ���Χ [0, 2PI)

		//double minVal, maxVal;
		//cv::Point minLoc, maxLoc;
		//// ������λͼ�е���Сֵ�����ֵ����λ��
		//cv::minMaxLoc(phase, &minVal, &maxVal, &minLoc, &maxLoc);

		//std::cout << "��λͼ��ֵ��Χ: [" << minVal << ", " << maxVal << "]" << std::endl;
		//std::cout << "��Сֵλ��: (" << minLoc.y << ", " << minLoc.x << ")" << std::endl;
		//std::cout << "���ֵλ��: (" << maxLoc.y << ", " << maxLoc.x << ")" << std::endl;

		m_PhiPhaseList[g] = phase.clone(); // ����˫����ԭʼ��λ
		phase = phase / (2 * CV_PI) * 255.0;
		phase.convertTo(fai_img[g], CV_8UC1);
	}
	//cv::Mat fai_img[3];
	//for (int g = 0; g < 3; ++g) {
	//	cv::Mat phase = cv::Mat::zeros(cossum[g].size(), CV_64F);

	//	for (int y = 0; y < phase.rows; ++y) {
	//		for (int x = 0; x < phase.cols; ++x) {
	//			double cos_val = cossum[g].at<double>(y, x);
	//			double sin_val = -sinsum[g].at<double>(y, x);  // ע���Ǹ���
	//			phase.at<double>(y, x) = std::atan2(sin_val, cos_val);  // ���� [-��, ��]
	//		}
	//	}

	//	m_PhiPhaseList[g] = phase.clone(); // ����˫����ԭʼ��λ

	//	// Ϊ�˿��ӻ����� [-��, ��] ӳ�䵽 [0, 255]
	//	cv::Mat phase_visual;
	//	phase_visual = (phase + CV_PI) / (2 * CV_PI) * 255.0;
	//	phase_visual.convertTo(fai_img[g], CV_8UC1);
	//}


	// ��ʾͼ��
	//for (int i = 0; i < 3; ++i) {
	//	std::string winname = "fai_img[" + std::to_string(i) + "]";
	//	cv::namedWindow(winname, 0);
	//	cv::resizeWindow(winname, grating_cols / 2, grating_rows / 2);
	//	cv::imshow(winname, fai_img[i]);
	//}
	//cv::waitKey(500);
}

// �������� cv::Mat��ÿ��Ԫ��ʹ�� std::round
cv::Mat roundMat(const cv::Mat& input)
{
	CV_Assert(input.type() == CV_64F || input.type() == CV_32F);

	cv::Mat result = cv::Mat::zeros(input.size(), input.type());

	input.forEach<double>([&](const double& val, const int* pos) -> void {
		result.at<double>(pos[0], pos[1]) = std::round(val);
		});

	return result;
}

void Grating::ThreeFrequencyUnwrap() {
	// ��ʼ�����ͼ��

	double f1 = T1, f2 = T2, f3 = T3;
	double freq12 = f1 - f2;
	double freq23 = f2 - f3;
	double freq123 = f1 - 2 * f2 + f3;

	for (int y = 0; y < grating_rows; ++y) {
		for (int x = 0; x < grating_cols; ++x) {
			double PH1 = m_PhiPhaseList[0].at<double>(y, x);
			double PH2 = m_PhiPhaseList[1].at<double>(y, x);
			double PH3 = m_PhiPhaseList[2].at<double>(y, x);

			auto delta_phase = [](double a, double b) {
				return (a > b) ? (a - b) : (a + 2 * CV_PI - b);
			};

			double PH12 = delta_phase(PH1, PH2);
			double PH23 = delta_phase(PH2, PH3);
			double PH123 = delta_phase(PH12, PH23);

			// �������λ����
			auto unwrap_phase = [](double deltaPh, double phWrap, double R) {
				return 2 * CV_PI * std::floor((deltaPh * R - phWrap) / (2 * CV_PI) + 0.5) + phWrap;
			};

			double ph23Unwrap = unwrap_phase(PH123, PH23, freq23 / freq123);
			double ph12Unwrap = unwrap_phase(PH123, PH12, freq12 / freq123);

			double phUnwrap1 = unwrap_phase(ph12Unwrap, PH1, f1 / freq12);
			// double phUnwrap2 = unwrap_phase(ph12Unwrap, PH2, f2 / freq12); // ��ѡ
			// double phUnwrap3 = unwrap_phase(ph23Unwrap, PH3, f3 / freq23); // ��ѡ

			m_AbsolutePhiPhase.at<double>(y, x) = phUnwrap1;
		}
	}
}



std::vector<std::vector<double>> Grating::threeFrequencyHeterodyneImproved(
	std::vector<cv::Point2f> cornerpoints_temp, vector<cv::Mat> grating_img) {
	/*cv::Mat camera_mat, distcoeffs_mat;
	getCameraParamFile(camera_mat, distcoeffs_mat);*/

	// cv::undistort(grating_imgs_before[i][j], grating_imgs[i][j], camera_mat,
	// distcoeffs_mat);

	//�Ĳ�����
	N_StepPhaseShifting(grating_img);

	//��Ƶ���
	ThreeFrequencyUnwrap();

	//��cornerpoints��Ӧ��theta����

	std::vector<double> samples_FAI_temp;

	for (int cornerpoints_num = 0; cornerpoints_num < cornerpoints_temp.size(); ++cornerpoints_num) {
		double temp =
			m_AbsolutePhiPhase.at<double>(cornerpoints_temp[cornerpoints_num].y,
				cornerpoints_temp[cornerpoints_num].x);

		samples_FAI_temp.push_back(temp);

		/*cv::circle(FAI_phase_img, cv::Point(cornerpoints_temp[cornerpoints_num].x,
		cornerpoints_temp[cornerpoints_num].y), 10, cv::Scalar(255, 255, 255), 5);*/
	}
	samples_FAI.push_back(samples_FAI_temp);

	/*cv::namedWindow("FAI_phase_img", 0);
	cv::resizeWindow("FAI_phase_img", cv::Size(grating_cols / 2, grating_rows /
	2)); cv::imshow("FAI_phase_img", FAI_phase_img);
	cv::imwrite("FAI_phase_img.bmp", FAI_phase_img);*/
	// cv::waitKey(10);

	return samples_FAI;
}
