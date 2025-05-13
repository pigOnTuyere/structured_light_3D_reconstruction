#include"CameraDlp.h"

#include<iostream>
#include"CameraDlp.h"
#include "Phaser.h"
#include "MyPointCloud.h"

using namespace std;



Eigen::MatrixXf CameraDlp::getCameraParamFile()//获取相机参数
{
	cv::Mat camera_mat, dist_coefficients;
	cv::FileStorage fin;
	fin.open("E:/cpp_project/struct-light/bin/cameraparam.xml", cv::FileStorage::READ);
	if (fin.isOpened())
	{
		fin["cameraMatrix"] >> camera_mat;
		fin["dist_coefficients"] >> dist_coefficients;
		fin.release();
	}
	else
	{
		cout << "no file" << endl;
		cout << "没找到cameraparam.xml" << endl;
	}
	Eigen::MatrixXf camera_matrix;
	cv::cv2eigen(camera_mat, camera_matrix);//将Opencv矩阵转换为Eigen矩阵

	return camera_matrix;
}


Eigen::MatrixXf CameraDlp::geta1_a8File()
{
	cv::Mat a_mat;
	Eigen::MatrixXf a_matrix;//存储从本地读取的a1-a8
	cv::FileStorage fin;
	fin.open("E:/cpp_project/struct-light/bin/a1_a8.xml", cv::FileStorage::READ);
	if (fin.isOpened())
	{
		fin["a1_a8"] >> a_mat;
		fin.release();
	}
	else
	{
		cout << " a1_a8.xml no file" << endl;
	}


	cv::cv2eigen(a_mat, a_matrix);//将Opencv矩阵转换为Eigen矩阵

	return a_matrix;
}


bool CameraDlp::detectCircleGridPoints(cv::Mat calib_img)
{
	// Invert the grayscale image to improve circle detection contrast
	cv::Mat calib_img_convert;
	cv::bitwise_not(calib_img, calib_img_convert);

	// Configure blob detector parameters
	cv::SimpleBlobDetector::Params params;
	//params.maxArea = 1e5;                 // Maximum blob area
	//params.minArea = 100;                 // Minimum blob area
	//params.minDistBetweenBlobs = 20;      // Minimum distance between blobs
	//params.minThreshold = 200;
	//params.maxThreshold = 245;
	//params.thresholdStep = 2;


	params.minThreshold = 10;
	params.maxThreshold = 220;
	params.thresholdStep = 5;

	params.filterByArea = true;
	params.minArea = 50;
	params.maxArea = 5000;

	params.filterByCircularity = true;
	params.minCircularity = 0.7f;

	params.filterByInertia = true;
	params.minInertiaRatio = 0.3f;

	params.filterByConvexity = true;
	params.minConvexity = 0.8f;

	// Create the blob detector
	cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

	corner_points_temp.clear(); // Clear previous temporary corner points

	// Detect circle grid points using symmetric pattern and blob detector
	bool circleflag = cv::findCirclesGrid(
		calib_img_convert,
		circleboard_size,
		corner_points_temp,
		cv::CALIB_CB_SYMMETRIC_GRID,
		blobDetector);

	// Return false if detection failed
	if (!circleflag) return false;

	// Refine corner locations to subpixel accuracy
	cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.001);
	cv::cornerSubPix(calib_img, corner_points_temp, cv::Size(7, 7), cv::Size(-1, -1), criteria);

	// Generate world coordinates for the detected grid
	std::vector<cv::Point3f> world_points_temp;
	for (int j = 0; j < circleboard_size.height; ++j) {
		for (int i = 0; i < circleboard_size.width; ++i) {
			world_points_temp.emplace_back(i * center_size, j * center_size, 0.0);
		}
	}

	// Store the detected image and world points
	corner_points.push_back(corner_points_temp);
	world_points.push_back(world_points_temp);

	return true;
}

#ifdef _WIN32
#include <direct.h>   // Windows: _mkdir
#define mkdir _mkdir
#else
#include <sys/stat.h> // Linux/Unix: mkdir
#include <sys/types.h>
#endif

#include <fstream>  // std::ofstream 用于简单验证文件是否可写

bool ensureDir(const std::string& path) {
#ifdef _WIN32
	return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
	return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

void CameraDlp::cameraCalibrationChessBoard()
{
	// Step 1: Camera calibration using chessboard pattern
	double reprojection_error = cv::calibrateCamera(
		world_points, corner_points,
		cv::Size(image_cols, image_rows),
		camera_matrix_cv, dist_coefficients, rotation_vectors, translation_vectors, 0);

	// Step 2: Convert camera matrix to Eigen format
	cv::cv2eigen(camera_matrix_cv, camera_matrix_eigen);

	// Step 3: Save intrinsic parameters
	if (!ensureDir(output_dir)) {
		std::cerr << "Failed to create output directory: " << output_dir << std::endl;
	}

	std::string intrinsic_path = output_dir + "/camera_parameters.xml";
	std::cout << "Extrinsic path: " << intrinsic_path << std::endl;
	cv::FileStorage fs(intrinsic_path, cv::FileStorage::WRITE);

	fs << "cameraMatrix" << camera_matrix_cv;
	fs << "distCoefficients" << dist_coefficients;
	fs.release();

	// Step 4: Save extrinsic parameters (rotation and translation)
	std::string extrinsic_path = output_dir + "/extrinsics_R_T.xml";
	std::cout << "Extrinsic path: " << extrinsic_path << std::endl;
	fs.open(extrinsic_path, cv::FileStorage::WRITE);
	for (size_t i = 0; i < rotation_vectors.size(); ++i)
	{
		cv::Mat rotation_matrix(3, 3, CV_64FC1);
		cv::Rodrigues(rotation_vectors[i], rotation_matrix);  // Convert to rotation matrix

		Eigen::Matrix3d rt;
		rt << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), translation_vectors[i].at<double>(0, 0),
			rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), translation_vectors[i].at<double>(1, 0),
			rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), translation_vectors[i].at<double>(2, 0);

		cv::Mat rt_cv(3, 3, CV_64FC1);
		cv::eigen2cv(rt, rt_cv);
		fs << "frame_" + std::to_string(i) << rt_cv;
	}
	fs.release();
}


void CameraDlp::calculateCameraPoints()
{
	const size_t frameCount = rotation_vectors.size();
	G.reserve(frameCount); // 预留空间避免频繁扩容

	// Step 1: 计算每幅图像对应的 G 矩阵
	for (size_t i = 0; i < frameCount; ++i)
	{
		// 将旋转向量转换为旋转矩阵 R
		cv::Mat R_mat;
		cv::Rodrigues(rotation_vectors[i], R_mat);

		// 构造 R1 R2 T 组成的 3x3 矩阵（只取前两列和T）
		Eigen::Matrix3d rt;
		rt << R_mat.at<double>(0, 0), R_mat.at<double>(0, 1), translation_vectors[i].at<double>(0, 0),
			R_mat.at<double>(1, 0), R_mat.at<double>(1, 1), translation_vectors[i].at<double>(1, 0),
			R_mat.at<double>(2, 0), R_mat.at<double>(2, 1), translation_vectors[i].at<double>(2, 0);

		// 计算 G = rt^-1 * K^-1
		Eigen::Matrix3d G_temp = rt.inverse() * camera_matrix_eigen.inverse();
		G.push_back(G_temp);
	}

	// Step 2: 计算每个角点的相机坐标
	const size_t imageCount = corner_points.size();
	camera_points.resize(imageCount); // 提前分配空间

	for (size_t i = 0; i < imageCount; ++i)
	{
		const size_t pointCount = corner_points[i].size();
		camera_points[i].reserve(pointCount); // 避免多次 reallocation

		const Eigen::Matrix3d& G_i = G[i];
		const Eigen::Matrix3d& K_inv = camera_matrix_eigen.inverse(); // 提前计算

		for (size_t j = 0; j < pointCount; ++j)
		{
			// 构造齐次像素坐标 (u, v, 1)
			Eigen::Vector3d pixel_homo;
			pixel_homo << corner_points[i][j].x, corner_points[i][j].y, 1.0;

			// 计算比例因子 s
			double s = pixel_homo.dot(G_i.row(2));

			// 反投影至相机坐标系并单位换算（mm → m）
			Eigen::Vector3d cam_point = (K_inv * pixel_homo) / s;
			cv::Point3f point_m(
				static_cast<float>(cam_point(0) / 1000.0),
				static_cast<float>(cam_point(1) / 1000.0),
				static_cast<float>(cam_point(2) / 1000.0)
			);

			camera_points[i].push_back(point_m);
		}
	}
}





void CameraDlp::phase3DPointsMapping(std::vector<std::vector<double>> samples_FAI)
{
	const int num_groups = static_cast<int>(samples_FAI.size());
	const int points_per_group = circleboard_size.width * circleboard_size.height;
	const int total_points = num_groups * points_per_group;

	if (camera_points.size() != num_groups) {
		std::cerr << "Error: Mismatch between samples_FAI and camera_points size.\n";
		return;
	}

	Eigen::MatrixXd Q(total_points, 8);  // 8 parameters to solve
	int row = 0;

	for (int group_idx = 0; group_idx < num_groups; ++group_idx) {
		const auto& group_camera_points = camera_points[group_idx];
		const auto& group_phases = samples_FAI[group_idx];

		if (group_camera_points.size() != group_phases.size()) {
			std::cerr << "Error: Mismatch in camera points and phase samples at group " << group_idx << ".\n";
			return;
		}

		for (int pt_idx = 0; pt_idx < group_camera_points.size(); ++pt_idx, ++row) {
			const cv::Point3f& pt = group_camera_points[pt_idx];
			const double theta = group_phases[pt_idx];

			Q(row, 0) = pt.x;
			Q(row, 1) = pt.y;
			Q(row, 2) = pt.z;
			Q(row, 3) = 1.0;
			Q(row, 4) = -pt.x * theta;
			Q(row, 5) = -pt.y * theta;
			Q(row, 6) = -pt.z * theta;
			Q(row, 7) = -theta;
		}
	}

	// SVD 求解
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXd a = svd.matrixV().col(7);  // 最小奇异值对应的解

	// 保存参数
	cv::Mat a_cv;
	cv::eigen2cv(a, a_cv);
	cv::FileStorage fs("a1_a8.xml", cv::FileStorage::WRITE);
	fs << "a1_a8" << a_cv;
	fs.release();
}


void CameraDlp::calculate3DPoints(cv::Mat FAI_phase, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat shadow_mask)
{
	// 获取相机和模型参数
	const Eigen::MatrixXf camera_mat = getCameraParamFile();
	const Eigen::MatrixXf a_mat = geta1_a8File();

	const float fx = camera_mat(0, 0), cx = camera_mat(0, 2);
	const float fy = camera_mat(1, 1), cy = camera_mat(1, 2);

	const float a1 = a_mat(0, 0), a2 = a_mat(1, 0), a3 = a_mat(2, 0), a4 = a_mat(3, 0);
	const float a5 = a_mat(4, 0), a6 = a_mat(5, 0), a7 = a_mat(6, 0), a8 = a_mat(7, 0);

	cloud->clear();

	for (int y = 0; y < image_rows; y += 2)
	{
		for (int x = 0; x < image_cols; x += 2)
		{
			if (shadow_mask.at<uchar>(y, x) == 0)
				continue;

			double phi = FAI_phase.at<double>(y, x);
			if (phi <= 0 || phi >= 140 * 2 * CV_PI)
				continue;

			float dx = x - cx;
			float dy = y - cy;

			float denominator = (a1 - phi * a5) * dx / fx +
				(a2 - phi * a6) * dy / fy +
				(a3 - phi * a7);

			if (std::abs(denominator) < 1e-6)
				continue;

			float Z = (phi * a8 - a4) / denominator;
			if (Z < 0.3f || Z > 0.8f)
				continue;

			float X = dx * Z / fx;
			float Y = dy * Z / fy;

			cloud->emplace_back(X, Y, Z);
		}
	}

	std::cout << "\n点云数量：" << cloud->size() << std::endl;
}


void CameraDlp::calculate3D_FeaturePoint(cv::Mat FAI_phase, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, cv::Mat shadowflag_img)
{

	Eigen::MatrixXf camera_matrix_eigen = getCameraParamFile();
	Eigen::MatrixXf a_matrix = geta1_a8File();

	cout << "\ncamera_matrix_eigen:" << endl;
	cout << camera_matrix_eigen << endl;
	cout << "\na_matrix:" << endl;
	cout << a_matrix << endl;


	for (int y = 650; y < 690; y += 2)
	{

		for (int x = 840; x < 880; x += 2)
		{
			if (shadowflag_img.at<uchar>(y, x) == 0)continue;

			double FAI_phase_temp = FAI_phase.at<double>(y, x);
			if (FAI_phase_temp <= 0 || FAI_phase_temp >= 140 * 2 * CV_PI)continue;

			double Zc = (FAI_phase_temp * a_matrix(7, 0) - a_matrix(3, 0)) /
				((a_matrix(0, 0) - FAI_phase_temp * a_matrix(4, 0)) * (x - camera_matrix_eigen(0, 2)) / camera_matrix_eigen(0, 0) +
					(a_matrix(1, 0) - FAI_phase_temp * a_matrix(5, 0)) * (y - camera_matrix_eigen(1, 2)) / camera_matrix_eigen(1, 1) +
					a_matrix(2, 0) - FAI_phase_temp * a_matrix(6, 0));

			double Xc = (x - camera_matrix_eigen(0, 2)) * Zc / camera_matrix_eigen(0, 0);

			double Yc = (y - camera_matrix_eigen(1, 2)) * Zc / camera_matrix_eigen(1, 1);

			//if (Zc < 0.4 || Zc>0.8)
			if (Zc < 0.3 || Zc>0.8)
			{
				continue;
			}
			src_cloud_ptr->push_back(pcl::PointXYZ(Xc, Yc, Zc));
			//cout << "x=" << Xc << "y=" << Yc << "z=" << Zc << endl;
		}

	}

	cout << "\n点云数量：" << src_cloud_ptr->size() << endl;


}



void CameraDlp::calculate3DPoints_TextureMapping(cv::Mat FAI_phase, cv::Mat source_picture, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_ptr, cv::Mat shadowflag_img)//纹理映射
{

	cout << "ceshi0" << endl;

	Eigen::MatrixXf camera_matrix_eigen = getCameraParamFile();
	Eigen::MatrixXf a_matrix = geta1_a8File();

	cout << "\ncamera_matrix_eigen:" << endl;
	cout << camera_matrix_eigen << endl;
	cout << "\na_matrix:" << endl;
	cout << a_matrix << endl;


	for (int y = 0; y < image_rows; y += 2)
	{

		for (int x = 0; x < image_cols; x += 2)
		{
			if (shadowflag_img.at<uchar>(y, x) == 0)continue;

			double FAI_phase_temp = FAI_phase.at<double>(y, x);
			if (FAI_phase_temp <= 0 || FAI_phase_temp >= 140 * 2 * CV_PI)continue;

			double Zc = (FAI_phase_temp * a_matrix(7, 0) - a_matrix(3, 0)) /
				((a_matrix(0, 0) - FAI_phase_temp * a_matrix(4, 0)) * (x - camera_matrix_eigen(0, 2)) / camera_matrix_eigen(0, 0) +
					(a_matrix(1, 0) - FAI_phase_temp * a_matrix(5, 0)) * (y - camera_matrix_eigen(1, 2)) / camera_matrix_eigen(1, 1) +
					a_matrix(2, 0) - FAI_phase_temp * a_matrix(6, 0));

			double Xc = (x - camera_matrix_eigen(0, 2)) * Zc / camera_matrix_eigen(0, 0);

			double Yc = (y - camera_matrix_eigen(1, 2)) * Zc / camera_matrix_eigen(1, 1);

			//if (Zc < 0.4 || Zc>0.8)
			if (Zc < 0.2 || Zc>0.9)
			{
				continue;
			}

			pcl::PointXYZRGB temp_point;
			temp_point.x = Xc;
			temp_point.y = Yc;
			temp_point.z = Zc;

			//uint8_t R = rand() % (256) + 0;
			//uint8_t G = rand() % (256) + 0;
			//uint8_t B = rand() % (256) + 0;

			//CvScalar scalar;
			//scalar = cvGet2D(source_picture, x, y);	//获取像素点的RGB颜色分量

			int b = source_picture.ptr<uchar>(y)[x * 3];
			int g = source_picture.ptr<uchar>(y)[x * 3 + 1];
			int r = source_picture.ptr<uchar>(y)[x * 3 + 2];

			temp_point.r = r;
			temp_point.g = g;
			temp_point.b = b;

			src_cloud_ptr->push_back(temp_point);
		}

	}

	cout << "\n点云数量：" << src_cloud_ptr->size() << endl;

	cv::namedWindow("TextureMapping", 0);
	cv::resizeWindow("TextureMapping", cv::Size(800, 600));
	//cv::imshow("TextureMapping", source_picture);

	cout << "ceshi" << endl;
}



