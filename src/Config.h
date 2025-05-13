#pragma once

#include <string>

struct Config {
	const std::string sep = "\\";
	const std::string root_dir = "E:\\cpp_project\\struct-light_up";
	const std::string data_dir = root_dir + sep + "data";
	// E:\结构光\danmu_nixiangji_duopin\danmu_逆相机\danmu_逆相机\biaoding\4bu3pin_hv706459_A130_160-1
	const std::string calib_dir = "E:\\结构光\\danmu_nixiangji_duopin\\danmu_逆相机\\danmu_逆相机\\biaoding\\4bu3pin_hv706459_A130_160-1";
	const std::string calib_file = calib_dir + sep + "stereoCalib.txt";

	const std::string project_dir = data_dir + sep + "protect";      // projection
	const std::string simu_dir = data_dir + sep + "simulation";      // simulation
	const std::string model_dir = data_dir + sep + "mouse";          // model

	const std::string output_dir = root_dir + sep + "outputs";
	const std::string output_dir_L = root_dir + sep + "outputs" + sep + "L";
	const std::string output_dir_R = root_dir + sep + "outputs" + sep + "R";
	const std::string save_file_point3d = output_dir + sep + "xyz.txt";

	bool write = true;
	bool show = true;

	const int num_groups = 15;
	const double A = 130;
	const double B = 90;
	const int N = 4;    // phase shifting steps
	const int T = 3;
	const double T1 = 70.;
	const double T2 = 64.;
	const double T3 = 59.;
	double center_size = 10;
	// Camera field of view
	const int W = 1600;
	const int H = 1200;
	const float B_min = 5;

	const int win = 3;
	const float pd = 0.5;
	const float min_z = 260;
	const float max_z = 305;
};
