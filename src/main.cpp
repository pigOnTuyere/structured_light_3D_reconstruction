/*
 * Author: pigOnTuyere
 * Email: 308128628@qq.com
 * Function: Monocular Structured Light + Phase Shift + Multi-frequency Heterodyne
 */

#include <iostream>
#include "CameraDlp.h"
#include "Phaser.h"
#include "MyPointCloud.h"
#include "Config.h"

int main()
{
        Config config;
        Phaser phaser(config);;
        CameraDlp camera_dlp(config);
        MyPointCloud point_cloud;

        // Load calibration data: reference images and phaser fringe images
        int num_images = config.N * config.T + 1;
        std::vector<cv::Mat> calibration_images;
        std::vector<std::vector<cv::Mat>> grating_calibration_images;

        for (int group_index = 1; group_index <= config.num_groups; group_index++)
        {
                std::string group_path = config.calib_dir + "/" + std::to_string(group_index) + "/*.bmp";
                std::cout << "Loading group path: " << group_path << std::endl;

                std::vector<cv::String> filenames;
                cv::glob(group_path, filenames, false);

                std::vector<cv::Mat> group_images;

                for (size_t i = 0; i < num_images; ++i)
                {
                        cv::Mat image = imread(filenames[i], IMREAD_GRAYSCALE);
                        if (image.empty()) {
                                std::cerr << "Failed to load image: " << filenames[i] << std::endl;
                                continue;
                        }

                        if (i == 0)
                        {
                                calibration_images.push_back(image);
                        }
                        else
                        {
                                group_images.push_back(image);
                        }

                        //std::cout << "Loaded: " << filenames[i] << std::endl;
                }
                std::cout << "Loaded group index: " << group_index << std::endl;
                grating_calibration_images.push_back(group_images);
        }

        std::cout << "Calibration image count: " << calibration_images.size() << std::endl;
        std::cout << "Phaser group count: " << grating_calibration_images.size() << std::endl;
        if (!grating_calibration_images.empty())
                std::cout << "Images in first phaser group: " << grating_calibration_images[0].size() << std::endl;

        // Extract calibration board features
        int num_successes = 0;
        for (int i = 0; i < config.num_groups; i++)
        {
                bool success = camera_dlp.detectCircleGridPoints(calibration_images[i]);
                std::cout << "Calibration attempt " << (i + 1) << (success ? " succeeded!" : " failed!") << std::endl;
                if (success) num_successes++;
        }

        // Perform camera calibration
        if (num_successes == config.num_groups)
        {
                camera_dlp.cameraCalibrationChessBoard();

                for (int i = 0; i < config.num_groups; ++i)
                {
                        phaser.threeFrequencyHeterodyneImproved(camera_dlp.corner_points[i], grating_calibration_images[i]);
                }

                camera_dlp.calculateCameraPoints();
                camera_dlp.phase3DPointsMapping(phaser.samples_FAI);

                std::cout << "System calibration completed!" << std::endl;
        }
        else
        {
                std::cout << "Calibration failed due to some unsuccessful data." << std::endl;
        }

        // Load reconstruction data
        //  E:\\结构光\\danmu_nixiangji_duopin\\danmu_逆相机\\danmu_逆相机\\biaoding\\4bu3pin_hv706459_A130_160-1\\1\\*.bmp
        std::string reconstruction_path = "E:\\结构光\\danmu_nixiangji_duopin\\danmu_逆相机\\danmu_逆相机\\biaoding\\4bu3pin_hv706459_A130_160-1\\1\\*.bmp";
        std::cout << reconstruction_path << std::endl;

        std::vector<cv::String> reconstruction_files;
        cv::Mat reference_image;
        std::vector<cv::Mat> grating_images;

        std::cout << "Loading reconstruction data files:" << std::endl;
        cv::glob(reconstruction_path, reconstruction_files, false);
        //std::cout << reconstruction_files.size() << std::endl;

        for (size_t i = 0; i < num_images; ++i)
        {
                if (i == 0)
                        reference_image = cv::imread(reconstruction_files[i]);
                else
                        grating_images.push_back(imread(reconstruction_files[i], IMREAD_GRAYSCALE));

                std::cout << reconstruction_files[i] << std::endl;
        }

        std::cout << "Loaded phaser images count: " << grating_images.size() << std::endl;

        // Phase unwrapping
        phaser.removeShadow(grating_images);
        phaser.N_StepPhaseShifting(grating_images);
        phaser.ThreeFrequencyUnwrap();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // 3D reconstruction
        camera_dlp.calculate3DPoints(phaser.m_absolute_phase, cloud_ptr, phaser.m_ShadowflagImage);
        camera_dlp.calculate3DPoints_TextureMapping(phaser.m_absolute_phase, reference_image, cloud_ptr_rgb, phaser.m_ShadowflagImage);

        pcl::io::savePCDFile("output.pcd", *cloud_ptr);

        // Display point cloud
        point_cloud.showSrcCloud(cloud_ptr, feature_cloud_ptr);
        // point_cloud.showSrcCloud_TextureMapping(cloud_ptr_rgb);

        cv::waitKey(0);
        return 0;
}
