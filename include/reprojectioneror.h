#ifndef REPROJECTIONERROR_H
#define REPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Dense>
#include "readfile.h"

#include "reprojection.h"

void cal_angleAxis_from_matrix( Eigen::Matrix4d& trans_matrix, double A[6])
{
    Eigen::AngleAxisd K;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> vector, displacement;
    
    R << trans_matrix(0,0), trans_matrix(0,1), trans_matrix(0,2),
         trans_matrix(1,0), trans_matrix(1,1), trans_matrix(1,2),
         trans_matrix(2,0), trans_matrix(2,1), trans_matrix(2,2);
    
    displacement << trans_matrix(0,3), trans_matrix(1,3), trans_matrix(2,3);

    K.fromRotationMatrix(R);
    //std::cout << " R: " << R << "\n" << "displacement: " << displacement << std::endl;
    //std::cout << "axis: " << K.axis() << " angle: " << K.angle() << std::endl;
    vector = K.angle() * K.axis();
    A[0] = vector(0,0);
    A[1] = vector(1,0);
    A[2] = vector(2,0);
    A[3] = displacement(0,0);
    A[4] = displacement(1,0);
    A[5] = displacement(2,0);

    //std::cout << "x: " << A[0] << " y: " << A[1] << " z: " << A[2] << " d1: " << A[3] << " d2: " << A[4] << " d3: "<< A[5] << std::endl;
    //std::cout << K.axis() << "\n\n" << K.angle() << std::endl;
    //std::cout << vector << std::endl;
}

void cal_euler_angles_from_matrix( Eigen::Matrix4d& trans_matrix, double* A)
{
    Eigen::Matrix<double, 3, 1> eulerAngle;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> vector, displacement;
    
    R << trans_matrix(0,0), trans_matrix(0,1), trans_matrix(0,2),
         trans_matrix(1,0), trans_matrix(1,1), trans_matrix(1,2),
         trans_matrix(2,0), trans_matrix(2,1), trans_matrix(2,2);
    
    displacement << trans_matrix(0,3), trans_matrix(1,3), trans_matrix(2,3);
    eulerAngle = R.eulerAngles(2,1,0);
    
    A[0] = eulerAngle(2,0);
    A[1] = eulerAngle(1,0);
    A[2] = eulerAngle(0,0);
    A[3] = displacement(0,0);
    A[4] = displacement(1,0);
    A[5] = displacement(2,0);
}

Eigen::Matrix4d cal_T_from_angleAxis(double angle_axis[6])
{
    Eigen::Matrix<double, 3, 1> axis;
    axis << angle_axis[0], angle_axis[1], angle_axis[2];
    double angle = sqrt(angle_axis[0]*angle_axis[0] + angle_axis[1]*angle_axis[1] + angle_axis[2]*angle_axis[2]);
    axis = axis / angle;
    Eigen::AngleAxisd w(angle, axis);
    Eigen::Matrix3d R;
    R = w.matrix();
    Eigen::Matrix4d T;
    T << R(0,0), R(0,1), R(0,2), angle_axis[3],
         R(1,0), R(1,1), R(1,2), angle_axis[4],
         R(2,0), R(2,1), R(2,2), angle_axis[5],
              0,      0,      0,             1;

    return T;  

}

void dot_product(double* v1, double* v2, double v3)
{
    v3 = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

void cross_product(double* v1, double* v2, double* v3)
{
    v3[0] = v1[1]*v2[2] - v2[1]*v1[2];
    v3[1] = v2[0]*v1[2] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v2[0]*v1[1];
}

/*
struct reprojectionerror
{
    reprojectionerror( std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>& paired_center_point )
    : _image_u(paired_center_point.second(0,0)), _image_v(paired_center_point.second(1,0)), _lidar_point(paired_center_point.first) {}

    bool operator() (const double* const param, double* residual) const
    {

        Eigen::Matrix4d T;
        T = cal_T_from_angleAxis(param);

        pointXYZRGB pt;
        pt = project_to_Cam_coor(_lidar_point, eigen_T, eigen_Cam_intrin, eigen_Cam_dist);
        double u = pt._uvz(0,0);
        double v = pt._uvz(1,0);

        residual[0] = (u-_image_u)*(u-_image_u) /* + (v-_image_v)*(v-_image_v);
        
        return true;
    }

    double _image_u;
    double _image_v;
    pointXYZRGB _lidar_point;
};

Eigen::Matrix4d ceres_solve( std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>>& paired_center_points )
    {

    ceres::Problem problem;
    double A[6];
    cal_angleAxis_from_matrix(eigen_T, A);

    for (int i = 0; i < paired_center_points.size(); i++)
    {
        ceres::CostFunction* cost_function = 
            new ceres::NumericDiffCostFunction<reprojectionerror, ceres::FORWARD, 1, 6>(
                new reprojectionerror(paired_center_points[i])
            );
        problem.AddResidualBlock(cost_function, NULL, A);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "a= " << A[0] << std::endl;
    std::cout << "b= " << A[1] << std::endl;
    std::cout << "c= " << A[2] << std::endl;

    Eigen::Matrix4d T;
    T = cal_T_from_angleAxis(A);
    return T;
}

struct error
{
    error(std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>& paired_center_point) 
    : point2d(paired_center_point.second), point3d(paired_center_point.first._point) {}

    template <typename T>
    bool operator() (const T* const camera, T* residuals) const
    {
        
        T p[3];
        T point[3];
        point[0] = T(point3d(0,0));
        point[1] = T(point3d(1,0));
        point[2] = T(point3d(2,0));

        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

        T x_cam1 = p[0]/p[2]; T y_cam1 = p[1]/p[2];  

        T u = eigen_Cam_intrin(0,0)*x_cam1 + eigen_Cam_intrin(0,2);
        T v = eigen_Cam_intrin(1,1)*y_cam1 + eigen_Cam_intrin(2,2);

        residuals[0] = u - T( point2d(0,0) );
        residuals[1] = v - T( point2d(1,0) );
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix<double, 3, 1> point2d,
     const Eigen::Matrix<double, 3, 1> point3d)
    {
        return (new ceres::AutoDiffCostFunction<error, 2, 6>
        (
            new error(paired_center_point)));
    }

    Eigen::Matrix<double, 3, 1> point2d;
    Eigen::Matrix<double, 3, 1> point3d;
};
*/

struct uv_error
{
    uv_error(Eigen::Matrix<double, 3, 1>& point3d, Eigen::Matrix<double, 2, 1>& point2d) 
    : _point3d(point3d), _image_u(point2d(0,0)), _image_v(point2d(1,0)) {}

    bool operator()(const double* const param, double* residuals) const
    {
        /*
        Eigen::Matrix4d T;
        double A[6] = {param[0],param[1],param[2],param[3],param[4],param[5]};
        T = cal_T_from_angleAxis(A);
        pointXYZRGB lidar_point;
        lidar_point._point = _point3d;
        lidar_point = project_to_Cam_coor(lidar_point, T, eigen_Cam_intrin, eigen_Cam_dist);

        double u = lidar_point._uvz(0,0);
        double v = lidar_point._uvz(1,0);

        residuals[0] = (u-_image_u)*(u-_image_u) + (v-_image_v)*(v-_image_v);
        */
        
        double v[3] = {_point3d(0,0), _point3d(1,0), _point3d(2,0)};
        double w[3] = {param[0], param[1], param[2]};
        double theta2 = w[0]*w[0] + w[1]*w[1] + w[2]*w[2];
        double theta = sqrt(theta2);
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double k[3] = {w[0]/theta, w[1]/theta, w[2]/theta};

        double v_dot_k = v[0]*k[0] + v[1]*k[1] + v[2]*k[2];

        double k_cross_v[3] = {
                              v[2] * k[1] - v[1] * k[2],
                              v[0] * k[2] - v[2] * k[0],
                              v[1] * k[0] - v[0] * k[1] 
                              };
   
        double v_rot[3];
        v_rot[0] = cos_theta*v[0] + (1.0 - cos_theta)*v_dot_k*k[0] + sin_theta*k_cross_v[0];
        v_rot[1] = cos_theta*v[1] + (1.0 - cos_theta)*v_dot_k*k[1] + sin_theta*k_cross_v[1];
        v_rot[2] = cos_theta*v[2] + (1.0 - cos_theta)*v_dot_k*k[2] + sin_theta*k_cross_v[2];

        v_rot[0] += param[3]; v_rot[1] += param[4]; v_rot[2] += param[5];

        double pu_cam1 = v_rot[0] / v_rot[2];
        double pv_cam1 = v_rot[1] / v_rot[2];

        double pu = eigen_Cam_intrin(0,0) * pu_cam1 + eigen_Cam_intrin(0,2);
        double pv = eigen_Cam_intrin(1,1) * pv_cam1 + eigen_Cam_intrin(1,2);

        residuals[0] = (pu-_image_u) * (pu-_image_u) + (pv-_image_v)*(pv-_image_v);
        //residuals[0] = (pu-_image_u) + (pv-_image_v);
        
        return true;
    };

    Eigen::Matrix<double, 3, 1> _point3d;
    double _image_u;
    double _image_v;
};

Eigen::Matrix4d ceres_solve( std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>>& paired_center_points)
{
    double camera[6];
    cal_angleAxis_from_matrix(eigen_T, camera);
    Eigen::Matrix<double, 3, 1> point3d;
    Eigen::Matrix<double, 2, 1> point2d;
    double fcost = 1000;
    Eigen::Matrix4d T;

    for (double m = 0; m < 11; m++)
    {
        camera[0] = (0.7 + m/20) * camera[0];

    for (double j = 0; j < 11; j++)
    {
        camera[1] = (0.7 + j/20) * camera[1];
    
    for (double k = 0; k < 11; k++)
    {
        camera[2] = (0.7 + k/20) * camera[2];
    
    ceres::Problem problem;
    for (int i = 0; i < paired_center_points.size(); i++)
    {
        point3d = paired_center_points[i].first._point;
        point2d << paired_center_points[i].second(0,0), paired_center_points[i].second(1,0);

        ceres::CostFunction* cost_function = 
            new ceres::NumericDiffCostFunction<uv_error, ceres::FORWARD, 1, 6>(new uv_error(point3d, point2d));

        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 camera
        );
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.final_cost < fcost)
    {
        fcost = summary.final_cost;
        std::cout << "cost: " << fcost << "-------------------- " << (100*m+10*j+k)/10 << "%" << std::endl;
        T = cal_T_from_angleAxis(camera);
    }
    

    }
    }
    }

    //summary.iterations;
    //std::cout << "final_cost " << summary.final_cost << std::endl;
    //std::cout << "param[0]: " << camera[0] << std::endl;
    //std::cout << "param[1]: " << camera[1] << std::endl;
    //std::cout << "param[2]: " << camera[2] << std::endl;
    //std::cout << "param[3]: " << camera[3] << std::endl;
    //std::cout << "param[4]: " << camera[4] << std::endl;
    //std::cout << "param[5]: " << camera[5] << std::endl;
            
    std::cout << "final_cost: " << fcost << std::endl;
    return T;

}

#endif