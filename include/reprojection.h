#ifndef REPROJECTION_H
#define REPROJECTION_H

#include "pointXYZRGB.h"
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Geometry>

pointXYZRGB project_to_Cam_coor
(pointXYZRGB lidar_point, Eigen::Matrix4d& eigen_T, Eigen::Matrix3d& eigen_Cam_intrin, Eigen::Matrix<double, 5, 1>& eigen_Cam_dist)
{  
        Eigen::Matrix<double, 4, 1> lidar_point_homogeneous;
        lidar_point_homogeneous << lidar_point._point(0,0), lidar_point._point(1,0) , lidar_point._point(2,0) , 1;
        
        Eigen::Matrix<double, 4, 1> lidar_point_cam =  eigen_T * lidar_point_homogeneous;

        Eigen::Matrix<double, 3, 1> lidar_point_cam1; 
        lidar_point_cam1 << lidar_point_cam(0,0)/lidar_point_cam(2,0), lidar_point_cam(1,0)/lidar_point_cam(2,0), lidar_point_cam(2,0)/lidar_point_cam(2,0);
        
        Eigen::Matrix<double, 3, 1> lidar_point_uv = eigen_Cam_intrin * lidar_point_cam1;
        lidar_point_uv(2,0) = lidar_point_cam(2,0);
  
        lidar_point._uvz = lidar_point_uv;

        double x_undirtorted = (lidar_point._uvz(0,0) - eigen_Cam_intrin(0,2)) / eigen_Cam_intrin(0,0);
        double y_undistorted = (lidar_point._uvz(1,0) - eigen_Cam_intrin(1,2)) / eigen_Cam_intrin(1,1);
        double x2 = x_undirtorted * x_undirtorted; double y2 = y_undistorted * y_undistorted;
        double xy = x_undirtorted * y_undistorted; double r2 = x2 + y2;

        double x_radial = x_undirtorted * (1 + eigen_Cam_dist(0,0) * r2 + eigen_Cam_dist(1,0) * r2 * r2);
        double y_radial = y_undistorted * (1 + eigen_Cam_dist(0,0) * r2 + eigen_Cam_dist(1,0) * r2 * r2);

        double x_tangential = 2 * eigen_Cam_dist(2,0) * xy + eigen_Cam_dist(3,0) * (r2 + 2 * x2);
        double y_tangential = 2 * eigen_Cam_dist(3,0) * xy + eigen_Cam_dist(2,0) * (r2 + 2 * y2);

        double x_distorted = x_radial + x_tangential;
        double y_distorted = y_radial + y_tangential;

        int u_distorted = x_distorted * eigen_Cam_intrin(0,0) + eigen_Cam_intrin(0,2);
        int v_distorted = y_distorted * eigen_Cam_intrin(1,1) + eigen_Cam_intrin(1,2);

        lidar_point._uvz(0,0) = u_distorted; lidar_point._uvz(1,0) = v_distorted;
        return lidar_point;
    
}

void fusion(std::vector<pointXYZRGB>& lidar_points, cv::Mat& image)
{
    for (int i = 0; i < lidar_points.size(); i++)
    {
        int u = lidar_points[i]._uvz(0,0);
        int v = lidar_points[i]._uvz(1,0);

        double z = lidar_points[i]._uvz(2,0) * 10;
        float r = (lidar_points[i]._reflection_rate / 255) * 1.5;
        if(0 <= u && u <= 2047 && 0 < v && v <= 1535)
        {   
  
            cv::Point center(u, v);
            cv::Mat gray(1,1,0, z); cv::Mat color;
            cv::applyColorMap(gray, color, cv::COLORMAP_JET);
            cv::Scalar _color = color.at<cv::Vec3b>(0,0);
            cv::circle(image, center, r+2, _color, -1);
        }
    }
    
 
}









#endif