/*
定义表示雷达数据点的结构体
_point: 点的空间位置（x,y,z);
_reflection_rate: 反射率；
_uvz: 经过变换后的像素位置(u,v,z), z为相机坐标系中空间点的z值。
_flag: 标志点所属的类别
*/
#ifndef POINTXYZRGB
#define POINTXYZRGB

#include <eigen3/Eigen/Dense>

struct pointXYZRGB
{
    Eigen::Matrix<double, 3, 1> _point;
    Eigen::Matrix<double, 3, 1> _uvz;
    double _reflection_rate;
    int flag = 0;
};

#endif
