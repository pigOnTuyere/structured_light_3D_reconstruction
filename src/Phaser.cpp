#include "Phaser.h"
#include <cmath>  // Include math header

void Phaser::removeShadow(const std::vector<cv::Mat>& phaser_img)
{
	const int N = static_cast<int>(phaser_img.size()) / 3;
	std::vector<double> sin_vals(N), cos_vals(N);

	// Precompute sin and cos values
	for (int k = 0; k < N; ++k) {
		sin_vals[k] = sin(2 * k * CV_PI / N);
		cos_vals[k] = cos(2 * k * CV_PI / N);
	}

	// Output modulation image container
	std::vector<cv::Mat> b_img(3);
	for (int i = 0; i < 3; ++i)
		b_img[i] = cv::Mat::zeros(image_rows, image_cols, CV_32FC1);

	const double mm = 2.0 / N;

	// Compute modulation for each frequency
	for (int f = 0; f < 3; ++f) {
		cv::Mat sin_sum = cv::Mat::zeros(image_rows, image_cols, CV_64FC1);
		cv::Mat cos_sum = cv::Mat::zeros(image_rows, image_cols, CV_64FC1);

		for (int k = 0; k < N; ++k) {
			cv::Mat tmp;
			phaser_img[f * N + k].convertTo(tmp, CV_64FC1);  // Convert to double precision

			sin_sum += tmp * sin_vals[k];
			cos_sum += tmp * cos_vals[k];
		}

		cv::Mat amplitude;
		cv::magnitude(sin_sum, cos_sum, amplitude);  // sqrt(sin^2 + cos^2)
		b_img[f] = mm * amplitude;  // Scale and store
	}

	// Shadow mask image
	m_ShadowflagImage = cv::Mat::ones(image_rows, image_cols, CV_8UC1) * 255;

	const float thresh_shadow = 2.0f;
	for (int f = 0; f < 3; ++f) {
		cv::Mat mask = b_img[f] <= thresh_shadow;
		m_ShadowflagImage.setTo(0, mask);
	}

	// Optional visualization
	// cv::imshow("ShadowMask", m_ShadowflagImage);
	// cv::waitKey(0);
}


void Phaser::N_StepPhaseShifting(const vector<cv::Mat>& phaser_img)
{
	const int N = phaser_img.size() / 3;
	std::vector<double> sin_table(N), cos_table(N);
	for (int k = 0; k < N; ++k) {
		double angle = 2.0 * M_PI * k / N;
		sin_table[k] = sin(angle);
		cos_table[k] = cos(angle);
	}

	// Prepare three groups of sin/cos accumulators (use CV_64FC1 for precision)
	cv::Mat sinsum[3], cossum[3];
	for (int i = 0; i < 3; ++i) {
		sinsum[i] = cv::Mat::zeros(image_rows, image_cols, CV_64FC1);
		cossum[i] = cv::Mat::zeros(image_rows, image_cols, CV_64FC1);
	}

	// Accumulate weighted results for each image group
	for (int k = 0; k < N; ++k) {
		for (int g = 0; g < 3; ++g) {
			cv::Mat img_double;
			phaser_img[k + g * N].convertTo(img_double, CV_64FC1); // Convert to double for calculation

			cv::Mat temp;
			cv::add(sinsum[g], img_double * sin_table[k], sinsum[g]);
			cv::add(cossum[g], img_double * cos_table[k], cossum[g]);
		}
	}

	// Calculate phase images and normalize to uchar image
	cv::Mat fai_img[3];
	for (int g = 0; g < 3; ++g) {
		cv::Mat phase;
		cv::phase(cossum[g], -sinsum[g], phase); // atan2(-sin, cos), range [0, 2PI)

		this->m_phase_List[g] = phase.clone(); // Save raw phase (double)
		phase = phase / (2 * CV_PI) * 255.0;
		phase.convertTo(fai_img[g], CV_8UC1);
	}

	//cv::Mat fai_img[3];
	//for (int g = 0; g < 3; ++g) {
	//	cv::Mat phase = cv::Mat::zeros(cossum[g].size(), CV_64F);

	//	for (int y = 0; y < phase.rows; ++y) {
	//		for (int x = 0; x < phase.cols; ++x) {
	//			double cos_val = cossum[g].at<double>(y, x);
	//			double sin_val = -sinsum[g].at<double>(y, x);
	//			phase.at<double>(y, x) = std::atan2(sin_val, cos_val);  // Keep values in [-¦Ð, ¦Ð]
	//		}
	//	}

	//	m_phase_list[g] = phase.clone(); // Save the original phase in double precision

	//	// For visualization, map [-¦Ð, ¦Ð] to [0, 255]
	//	cv::Mat phase_visual;
	//	phase_visual = (phase + CV_PI) / (2 * CV_PI) * 255.0;
	//	phase_visual.convertTo(fai_img[g], CV_8UC1);
	//}

	// Optional visualization
	// for (int i = 0; i < 3; ++i) {
	//     std::string winname = "fai_img[" + std::to_string(i) + "]";
	//     cv::namedWindow(winname, 0);
	//     cv::resizeWindow(winname, image_cols / 2, image_rows / 2);
	//     cv::imshow(winname, fai_img[i]);
	// }
	// cv::waitKey(500);
}

// Round elements of cv::Mat using std::round
cv::Mat roundMat(const cv::Mat& input)
{
	CV_Assert(input.type() == CV_64F || input.type() == CV_32F);

	cv::Mat result = cv::Mat::zeros(input.size(), input.type());

	input.forEach<double>([&](const double& val, const int* pos) -> void {
		result.at<double>(pos[0], pos[1]) = std::round(val);
		});

	return result;
}

void Phaser::ThreeFrequencyUnwrap() {
	// Initialize output image

	double f1 = T1, f2 = T2, f3 = T3;
	double freq12 = f1 - f2;
	double freq23 = f2 - f3;
	double freq123 = f1 - 2 * f2 + f3;

	for (int y = 0; y < image_rows; ++y) {
		for (int x = 0; x < image_cols; ++x) {
			double PH1 = this->m_phase_List[0].at<double>(y, x);
			double PH2 = this->m_phase_List[1].at<double>(y, x);
			double PH3 = this->m_phase_List[2].at<double>(y, x);

			auto delta_phase = [](double a, double b) {
				return (a > b) ? (a - b) : (a + 2 * CV_PI - b);
			};

			double PH12 = delta_phase(PH1, PH2);
			double PH23 = delta_phase(PH2, PH3);
			double PH123 = delta_phase(PH12, PH23);

			// Phase unwrapping function
			auto unwrap_phase = [](double deltaPh, double phWrap, double R) {
				return 2 * CV_PI * std::floor((deltaPh * R - phWrap) / (2 * CV_PI) + 0.5) + phWrap;
			};

			double ph23Unwrap = unwrap_phase(PH123, PH23, freq23 / freq123);
			double ph12Unwrap = unwrap_phase(PH123, PH12, freq12 / freq123);

			double phUnwrap1 = unwrap_phase(ph12Unwrap, PH1, f1 / freq12);
			// double phUnwrap2 = unwrap_phase(ph12Unwrap, PH2, f2 / freq12); // Optional
			// double phUnwrap3 = unwrap_phase(ph23Unwrap, PH3, f3 / freq23); // Optional

			this->m_absolute_phase.at<double>(y, x) = phUnwrap1;
		}
	}
}

std::vector<std::vector<double>> Phaser::threeFrequencyHeterodyneImproved(
	std::vector<cv::Point2f> cornerpoints_temp, vector<cv::Mat> grating_img) {

	// Step phase shifting
	N_StepPhaseShifting(grating_img);

	// Three-frequency heterodyne unwrapping
	ThreeFrequencyUnwrap();

	// Extract theta samples at cornerpoints

	std::vector<double> samples_FAI_temp;

	for (int cornerpoints_num = 0; cornerpoints_num < cornerpoints_temp.size(); ++cornerpoints_num) {
		double temp =
			m_absolute_phase.at<double>(cornerpoints_temp[cornerpoints_num].y,
				cornerpoints_temp[cornerpoints_num].x);

		samples_FAI_temp.push_back(temp);

		// Optional: draw sample point
		// cv::circle(FAI_phase_img, cv::Point(cornerpoints_temp[cornerpoints_num].x,
		// cornerpoints_temp[cornerpoints_num].y), 10, cv::Scalar(255, 255, 255), 5);
	}
	samples_FAI.push_back(samples_FAI_temp);

	// Optional visualization
	// cv::namedWindow("FAI_phase_img", 0);
	// cv::resizeWindow("FAI_phase_img", cv::Size(image_cols / 2, image_rows / 2));
	// cv::imshow("FAI_phase_img", FAI_phase_img);
	// cv::imwrite("FAI_phase_img.bmp", FAI_phase_img);
	// cv::waitKey(10);

	return samples_FAI;
}