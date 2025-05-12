#pragma once

struct Config {
	const string sep = "/";
	const string root_dir = "E:\cpp_project\struct-light_up";
	const string data_dir = root_dir + sep + "data";
	const string calib_dir = "L:\\三维重建课程\\八参数标定重建示例数据\\20240517\\标定";
	const string calib_file = calib_dir + sep + "stereoCalib.txt";

	const string project_dir = data_dir + sep + "protect";      // 投影
	const string simu_dir = data_dir + sep + "simulation";   // 仿真
	const string model_dir = data_dir + sep + "mouse";        // 测量

	const string output_dir = root_dir + sep + "outputs";
	const string output_dir_L = root_dir + sep + "outputs" + sep + "L";
	const string output_dir_R = root_dir + sep + "outputs" + sep + "R";
	const string save_file_point3d = output_dir + sep + "xyz.txt";

	bool         write = true;
	bool         show = true;

	const int num_groups = 15;
	const double A = 130;
	const double B = 90;
	const int    N = 8;    // 相移步数
	const int    T = 3;
	const double T1 = 28.;
	const double T2 = 26.;
	const double T3 = 24.;
	//Camera field of view
	const int    W = 1600;  
	const int    H = 1200;
	const float  B_min = 5;

	// 
	const int    win = 3;
	const float  pd = 0.5;
	const float  min_z = 260;
	const float  max_z = 305;
};