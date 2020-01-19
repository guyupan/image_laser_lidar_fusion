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

#include "readfile.h"
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "pointXYZRGB.h"




//从txt文件中读取点云数据，输入文件名; 返回元素为点云的向量。
std::vector<pointXYZRGB> readpointcloud( std::string& filename );


//保存点云数据到txt文件。
void savepointcloud(std::vector<pointXYZRGB>& lidar_points);

//读取激光雷达到相机的外参，返回4×4矩阵。
Eigen::Matrix4d readCalibLidarToCam(std::string& filename);

//读取相机内参，返回3×3矩阵。
Eigen::Matrix3d readCamintrin(std::string& filename);

//读取相机畸变系数，返回5*1矩阵。
Eigen::Matrix<double, 5, 1> readCamdist(std::string& filename);





#endif