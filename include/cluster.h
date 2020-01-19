#ifndef CLUSTER_H
#define CLUSTER_H

#include <ceres/ceres.h>
#include "pointXYZRGB.h"

//求绝对值。
float abs_value(float x);

//对不同反射率的点云聚类。
std::vector<pointXYZRGB> clustering_1(std::vector<pointXYZRGB>& lidar_points, float& w0, float w1, float w2);

//求两点之间的距离。
double distance(pointXYZRGB& point1, pointXYZRGB& point2);

//求线之间的高度。
double height_distance(pointXYZRGB& point1, pointXYZRGB& point2);

//对不同反光板上的点云聚类。
void meanshift(pointXYZRGB center_point, std::vector<pointXYZRGB>& lidar_points,int& flag); 

//对不同线上的点云聚类。
void clustering_line(std::vector<pointXYZRGB>& lidar_points, int flag, std::vector<pointXYZRGB>& circle_points);

//提取圆心。
void extracting_circle_center(std::vector<pointXYZRGB>& lidar_points, int& classes, std::vector<pointXYZRGB>& circle_points);

//对不同的圆心分类；
void clustering_2(std::vector<pointXYZRGB>& lidar_points, int& classes);


struct FITTING_PLANE_COST
{
    FITTING_PLANE_COST(double x, double y, double z, pointXYZRGB& center_point): _x(x), _y(y), _z(z), _center_point(center_point){}
    template <typename T>
    bool operator()(const T* const abc, T* residual)const
    {
        residual[0] = (abc[0]*_x + abc[1]*_y + abc[2]*_z - abc[0]*_center_point._point(0,0) - abc[1]*_center_point._point(1,0) 
        - abc[2]*_center_point._point(2,0)) * (abc[0]*_x + abc[1]*_y + abc[2]*_z - abc[0]*_center_point._point(0,0) - abc[1]*_center_point._point(1,0) 
        - abc[2]*_center_point._point(2,0)) /(abc[0]*abc[0] + abc[1]*abc[1] * abc[2]*abc[2])    ;
        return true;
    }

    const double _x, _y, _z;
    const pointXYZRGB _center_point;
};

struct  FITTING_CIRCLRCENTER_COST
{
    FITTING_CIRCLRCENTER_COST(double x, double y):_x(x), _y(y) {}
    template <typename T>
    bool operator()(const T* const abr, T* residual)const
    {       
        
        residual[0] = ((_x-abr[0])*(_x-abr[0])+(_y-abr[1])*(_y-abr[1])-abr[2]*abr[2])*((_x-abr[0])*(_x-abr[0])+(_y-abr[1])*(_y-abr[1])-abr[2]*abr[2]);
        return true;
    }
    double _x, _y;
};

//叉乘。
Eigen::Matrix<double, 3, 1> x ( Eigen::Matrix<double,3,1>& point1, Eigen::Matrix<double,3,1>& point2);

//点乘。
double dot(Eigen::Matrix<double, 3, 1>& vector1, Eigen::Matrix<double, 3, 1>& vector2);

//求向量长度。
double norm(Eigen::Matrix<double, 3, 1>& vector);

//非线性求解平面位置
Eigen::Matrix<double, 3, 1> ceres_solve_plane(std::vector<pointXYZRGB>& lidar_points, pointXYZRGB& center_point);

//非线性求解圆心位置
Eigen::Matrix<double, 3, 1> ceres_solve_circlecenter(std::vector<pointXYZRGB>& temp_points, pointXYZRGB& center_point);

//提取圆心
pointXYZRGB extracting_circle(std::vector<pointXYZRGB>& lidar_points);

//提取圆心
std::vector<pointXYZRGB> extracting_circle_center(std::vector<pointXYZRGB>& lidar_points, int& classes);

//求像素距离。
double uv_distance(pointXYZRGB& lidar_center_point, Eigen::Matrix<double, 2, 1>& image_center_point);

//配对圆心
std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>> pair_center_point(pointXYZRGB& lidar_center_point, std::vector<Eigen::Matrix<double, 2, 1>>& image_center_point);

//配对圆心
void pair_center_points(std::vector<pointXYZRGB>& lidar_center_points, std::vector<Eigen::Matrix<double, 2, 1>>& image_center_points, 
std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>>& paired_center_points);


#endif