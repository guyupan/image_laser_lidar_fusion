/*
读取数据的函数
eigen_T: 变换矩阵
        [R, t
         0, 1]

eigen_Cam_intrin: 相机内参
        [fx, 0, cx
          0,fy, cy
          0, 0, 1]

eigen_Cam_dist: 相机畸变参数；
        [k1, k2, k3, k4, k5]

lidar_points: 点云数据集
*/

#ifndef READFILE_H
#define READFILE_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "pointXYZRGB.h"

extern Eigen::Matrix4d eigen_T;
extern Eigen::Matrix3d eigen_Cam_intrin;
extern Eigen::Matrix<double, 5, 1> eigen_Cam_dist;
extern std::vector<pointXYZRGB> lidar_points;

//从txt文件中读取点云数据，输入文件名; 返回元素为点云的向量。
std::vector<pointXYZRGB> readpointcloud( std::string& filename );


//保存点云数据到txt文件。
void savepointcloud(std::vector<pointXYZRGB>& lidar_points);

//读取激光雷达到相机的外参，返回4×4矩阵。
Eigen::Matrix4d readCalibLidarToCam(std::string& filename);

//读取相机内参，返回3×3矩阵。
Eigen::Matrix3d readCamintrin(std::string& filename)；

//读取相机畸变系数，返回5*1矩阵。
Eigen::Matrix<double, 5, 1> readCamdist(std::string& filename)；


std::vector<pointXYZRGB> readpointcloud( std::string& filename )
{   
    std::vector<pointXYZRGB> lidar_points;
    pointXYZRGB lidar_point;
    std::ifstream pointcloud;
    pointcloud.open(filename);
    if(pointcloud.is_open())
    {
        std::cout << "load " << filename << "..." << std::endl;
    }
    else
    {
        std::cout << "ERROR: can't load " << filename << std::endl;
    }

    std::string line;

    while (getline(pointcloud, line))
    {
        std::istringstream record(line);
        double x, y, z;
        if( record >> x >> y >> z >> lidar_point._reflection_rate )
        { 
        lidar_point._point << x, y, z;
        lidar_points.push_back(lidar_point);
    }
    else
    {
        lidar_point._point << 0.0, 0.0, 0.0;
        lidar_point._reflection_rate = 0.0;
        lidar_points.push_back(lidar_point); 
    }
}
    std::cout << "lidar_points.size: " << lidar_points.size() << std::endl;
    return lidar_points;
}

void savepointcloud(std::vector<pointXYZRGB>& lidar_points)
{
    for (int i = 0; i < lidar_points.size(); i++)
    {
       lidar_points[i]._reflection_rate = lidar_points[i].flag*15; 
    }
    
    std::ofstream outfile("out.txt", std::ios::trunc);
    for (int i = 0; i < lidar_points.size(); i++)
    {
        outfile << lidar_points[i]._point(0,0) << " " << lidar_points[i]._point(1,0) << " " << lidar_points[i]._point(2,0) << " " 
        << lidar_points[i]._reflection_rate << " " << lidar_points[i].flag << "\n";
    }
    outfile.close();
}

Eigen::Matrix4d readCalibLidarToCam(std::string& filename)
{
    Eigen::Matrix4d eigen_T;
    cv::FileStorage CalibLidarToCam(filename, cv::FileStorage::READ);
    if (!CalibLidarToCam.isOpened())
    {
        std::cerr << "ERROR: Wrong path to calibration file" << std::endl;
    }
    cv::Mat cv_T;
    CalibLidarToCam["lidar_to_cam"] >> cv_T;
    cv::cv2eigen(cv_T, eigen_T);
    //std::cout << "eigen_T " << eigen_T << std::endl;
    return eigen_T;
}

Eigen::Matrix3d readCamintrin(std::string& filename)
{
    Eigen::Matrix3d eigen_Cam_intrin;
    cv::FileStorage CalibCamToCam(filename, cv::FileStorage::READ);
    if (!CalibCamToCam.isOpened())
    {
        std::cerr << "ERROR: Wrong path to calibration file" << std::endl;
    }
    cv::Mat cv_CamMat;
    CalibCamToCam["CameraMatrix"] >> cv_CamMat;
    
    cv::cv2eigen(cv_CamMat, eigen_Cam_intrin);
    return eigen_Cam_intrin;
}

Eigen::Matrix<double, 5, 1> readCamdist(std::string& filename)
{
    Eigen::Matrix<double, 5, 1> eigen_Cam_dist;
    cv::FileStorage CalibCamToCam(filename, cv::FileStorage::READ);
    if (!CalibCamToCam.isOpened())
    {
        std::cerr << "ERROR: Wrong path to calibration file" << std::endl;
    }
    cv::Mat cv_CamDist;
    CalibCamToCam["CameraDistCoffes"] >> cv_CamDist;

    cv::cv2eigen(cv_CamDist, eigen_Cam_dist);
    return eigen_Cam_dist;
}
#endif