#include <iostream>

#include "pointXYZRGB.h"
#include "image_process.h"
#include "readfile.h"
#include "cluster.h"
#include "reprojection.h"
#include "reprojectioneror.h"

void pairing_center_points( std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>>& paired_center_points )
{
    for (int i = 0; i < 5; i++)
    {

    float w0 = 90.0; float w1 = 100.0; float w2 = 110.0;
    int classes;
    //double z_max , z_min ;
    cv::Mat image;
    std::vector<pointXYZRGB>  lidar_points, center_points, circle_points, lidar_center_points, clustered_lidar_points;
    std::vector<Eigen::Matrix<double, 2, 1>> image_center_points;  

    int n = i;
    std::string lidar_points_file = "./demo(1)/lidar_right/";
    std::string image_file = "_L.png";
    std::string str1, str2, str3;
    str1 = "00"; str2 = ".txt", str3 = "demo(1)/hk_right/00";
    lidar_points_file += str1 + std::to_string(n) + str2;
    image_file = str3 + std::to_string(n) + image_file;

    lidar_points = readpointcloud(lidar_points_file);
    image = cv::imread(image_file);

    clustered_lidar_points = clustering_1(lidar_points, w0, w1, w2); //对不同反射率的点云聚类    
    clustering_2(clustered_lidar_points, classes);    //对不同板上的点云聚类
    extracting_circle_center(clustered_lidar_points, classes, circle_points);//提取圆上的点云
    clustering_2(clustered_lidar_points, classes);    //对不同板上的点云聚类
    lidar_center_points = extracting_circle_center(clustered_lidar_points, classes);    //提取圆心
    
    std::vector<pointXYZRGB> temp;
    for (int i = 0; i < lidar_points.size(); i++)
    {
        lidar_points[i] = project_to_Cam_coor(lidar_points[i], eigen_T, eigen_Cam_intrin, eigen_Cam_dist);
        if (lidar_points[i]._uvz(2,0) > 0)
        {
            temp.push_back(lidar_points[i]);
        }
    }
    lidar_points = temp;
    temp.clear();

    for (int i = 0; i < lidar_center_points.size(); i++)
    {
        lidar_center_points[i] = project_to_Cam_coor(lidar_center_points[i], eigen_T, eigen_Cam_intrin, eigen_Cam_dist);
        if (lidar_center_points[i]._uvz(2,0) > 0)
        {
            temp.push_back(lidar_center_points[i]);
        }
    }
    lidar_center_points = temp;
    savepointcloud(lidar_center_points);

    image_center_points = find_circle_centers(image);
    pair_center_points(lidar_center_points, image_center_points, paired_center_points);
    //fusion(lidar_points, image);
    //fusion(lidar_center_points, image);
    }
}

void test()
{
    
    std::string calib_lidar = "right_lidar_to_right_camera.yml";
    std::string calib_cam = "camera/hk_right_params.yml";
    

    eigen_Cam_intrin = readCamintrin(calib_cam);
    eigen_Cam_dist = readCamdist(calib_cam);
    eigen_T = readCalibLidarToCam(calib_lidar);

    std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>> paired_center_points;

    pairing_center_points(paired_center_points);

    std::cout << paired_center_points.size() << std::endl;


    Eigen::Matrix4d T;
    T = ceres_solve(paired_center_points);
    
    

    //std::cout << "\n" << T << std::endl;

    
    for (int i = 0; i < 1; i++)
    {
    std::vector<pointXYZRGB>  lidar_points;
    Mat image;
    int n = 1;
    std::string lidar_points_file = "./demo(1)/lidar_right/";
    std::string image_file = "_L.png";
    std::string str1, str2, str3;
    str1 = "00"; str2 = ".txt", str3 = "demo(1)/hk_right/00";
    lidar_points_file += str1 + std::to_string(n) + str2;
    image_file = str3 + std::to_string(n) + image_file;

    lidar_points = readpointcloud(lidar_points_file);
    image = cv::imread(image_file);

    std::vector<pointXYZRGB> temp;
    for (int i = 0; i < lidar_points.size(); i++)
    {
        lidar_points[i] = project_to_Cam_coor(lidar_points[i], T, eigen_Cam_intrin, eigen_Cam_dist);
        if (lidar_points[i]._uvz(2,0) > 0)
        {
            temp.push_back(lidar_points[i]);
        }
    }
    lidar_points = temp;
    temp.clear(); 
    fusion(lidar_points, image);
    cv::namedWindow("image", 0);
    cv::imshow("image", image);
    cv::waitKey( 0 );
    }

}


void test1()
{
    int n = 1;
    std::string lidar_points_file = "./lidar_right/";
    std::string image_file = "_L.png";
    std::string str1, str2, str3;
    str1 = "00"; str2 = ".txt", str3 = "hk_right/00";
    lidar_points_file += str1 + std::to_string(n) + str2;
    image_file = str3 + std::to_string(n) + image_file;

    std::vector<pointXYZRGB> lidar_points;
    cv::Mat image;
    lidar_points = readpointcloud(lidar_points_file);
    image = cv::imread(image_file);

    std::string calib_lidar = "right_lidar_to_right_camera.yml";
    std::string calib_cam = "./camera/hk_right_params.yml";
    eigen_Cam_intrin = readCamintrin(calib_cam);
    eigen_T = readCalibLidarToCam(calib_lidar);

    double param[6];
    cal_angleAxis_from_matrix(eigen_T, param);
    std::cout << "x: " << param[0] << " y: " << param[1] << " z: " << param[2] << " d1: " << param[3] << " d2: " << param[4] << " d3: "<< param[5] << std::endl;
    std::vector<pointXYZRGB> temp1;
    for (int i = 0; i < lidar_points.size(); i++)
    {   
         
        Eigen::Matrix<double, 3, 1> _point3d;
        _point3d = lidar_points[i]._point;
        double v[3] = {_point3d(0,0), _point3d(1,0), _point3d(2,0)};

        double w[3] = {param[0], param[1], param[2]};
        double theta2 = w[0]*w[0] + w[1]*w[1] + w[2]*w[2];
        double theta = sqrt(theta2);
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double k[3] = {w[0]/theta, w[1]/theta, w[2]/theta};

        double v_dot_k = v[0]*k[0] + v[1]*k[1] + v[2]*k[2];

        double temp = v_dot_k * (1.0 - cos_theta);
        double k_cross_v[3] = {
                              v[2] * k[1] - v[1] * k[2],
                              v[0] * k[2] - v[2] * k[0],
                              v[1] * k[0] - v[0] * k[1] 
                              };
        
        double v_rot[3];
        v_rot[0] = cos_theta * v[0] + temp * k[0] + sin_theta * k_cross_v[0];
        v_rot[1] = cos_theta * v[1] + temp * k[1] + sin_theta * k_cross_v[1];
        v_rot[2] = cos_theta * v[2] + temp * k[2] + sin_theta * k_cross_v[2];

        if (v_rot[2] > 0)
        {
        v_rot[0] += param[3]; v_rot[1] += param[4]; v_rot[2] += param[5];
                
        double pu_cam1 = v_rot[0] / v_rot[2];
        double pv_cam1 = v_rot[1] / v_rot[2];

        double pu = eigen_Cam_intrin(0,0) * pu_cam1 + eigen_Cam_intrin(0,2);
        double pv = eigen_Cam_intrin(1,1) * pv_cam1 + eigen_Cam_intrin(1,2);

        lidar_points[i]._uvz(0,0) = pu;
        lidar_points[i]._uvz(1,0) = pv;
        lidar_points[i]._uvz(2,0) = v_rot[0];
        temp1.push_back(lidar_points[i]);
        }
        //std::cout << v_rot[0] << " " << v_rot[1] << " " << v_rot[2] << std::endl;
    }
    lidar_points = temp1;

    std::cout << "eigen_Cam_intrin: \n" << eigen_Cam_intrin << std::endl;
    //savepointcloud(lidar_points);
    fusion(lidar_points, image);

    namedWindow("image", WINDOW_NORMAL);
    imshow("image", image);
    waitKey(0);
}

void test2()
{
    
    
    
    std::string calib_lidar = "right_lidar_to_right_camera.yml";
    std::string calib_cam = "camera/hk_right_params.yml";
    cv::Mat image;

    eigen_Cam_intrin = readCamintrin(calib_cam);
    eigen_Cam_dist = readCamdist(calib_cam);
    eigen_T = readCalibLidarToCam(calib_lidar);
    std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>> paired_center_points;

   
    for (int i = 0; i < 5; i++)
    {

    float w0 = 90.0; float w1 = 100.0; float w2 = 110.0;
    int classes;
    double z_max , z_min ;
    std::vector<pointXYZRGB>  lidar_points, center_points, circle_points, lidar_center_points, clustered_lidar_points;
    std::vector<Eigen::Matrix<double, 2, 1>> image_center_points;  

    int n = i;
    std::string lidar_points_file = "lidar_right/";
    std::string image_file = "_L.png";
    std::string str1, str2, str3;
    str1 = "00"; str2 = ".txt", str3 = "hk_right/00";
    lidar_points_file += str1 + std::to_string(n) + str2;
    image_file = str3 + std::to_string(n) + image_file;

    lidar_points = readpointcloud(lidar_points_file);
    image = cv::imread(image_file);

    clustered_lidar_points = clustering_1(lidar_points, w0, w1, w2); //对不同反射率的点云聚类    
    clustering_2(clustered_lidar_points, classes);    //对不同板上的点云聚类
    extracting_circle_center(clustered_lidar_points, classes, circle_points);//提取圆上的点云
    clustering_2(clustered_lidar_points, classes);    //对不同板上的点云聚类
    lidar_center_points = extracting_circle_center(clustered_lidar_points, classes);    //提取圆心
    
    std::vector<pointXYZRGB> temp;
    for (int i = 0; i < lidar_points.size(); i++)
    {
        lidar_points[i] = project_to_Cam_coor(lidar_points[i], eigen_T, eigen_Cam_intrin, eigen_Cam_dist);
        if (lidar_points[i]._uvz(2,0) > 0)
        {
            temp.push_back(lidar_points[i]);
        }
    }
    lidar_points = temp;
    temp.clear();

    for (int i = 0; i < lidar_center_points.size(); i++)
    {
        lidar_center_points[i] = project_to_Cam_coor(lidar_center_points[i], eigen_T, eigen_Cam_intrin, eigen_Cam_dist);
        if (lidar_center_points[i]._uvz(2,0) > 0)
        {
            temp.push_back(lidar_center_points[i]);
        }
    }
    lidar_center_points = temp;
    savepointcloud(lidar_center_points);

    image_center_points = find_circle_centers(image);
    pair_center_points(lidar_center_points, image_center_points, paired_center_points);
    fusion(lidar_points, image);
    fusion(lidar_center_points, image);
    }

    std::cout << paired_center_points.size() << std::endl;

    
    

    
    Eigen::Matrix4d T;
    T = ceres_solve(paired_center_points);
    
    

    //std::cout << "\n" << T << std::endl;

    
    for (int i = 0; i < 1; i++)
    {
    std::vector<pointXYZRGB>  lidar_points;
    int n = i;
    std::string lidar_points_file = "lidar_right/";
    std::string image_file = "_L.png";
    std::string str1, str2, str3;
    str1 = "00"; str2 = ".txt", str3 = "hk_right/00";
    lidar_points_file += str1 + std::to_string(n) + str2;
    image_file = str3 + std::to_string(n) + image_file;

    lidar_points = readpointcloud(lidar_points_file);
    image = cv::imread(image_file);

    std::vector<pointXYZRGB> temp;
    for (int i = 0; i < lidar_points.size(); i++)
    {
        lidar_points[i] = project_to_Cam_coor(lidar_points[i], T, eigen_Cam_intrin, eigen_Cam_dist);
        if (lidar_points[i]._uvz(2,0) > 0)
        {
            temp.push_back(lidar_points[i]);
        }
    }
    lidar_points = temp;
    temp.clear(); 
    fusion(lidar_points, image);
    }


    cv::namedWindow("image", 0);
    cv::imshow("image", image);
    cv::waitKey( 0 );
   
}

void test3()
{
    int n = 1;
    std::string lidar_points_file = "./lidar_right/";
    std::string image_file = "_L.png";
    std::string str1, str2, str3;
    str1 = "00"; str2 = ".txt", str3 = "hk_right/00";
    lidar_points_file += str1 + std::to_string(n) + str2;
    image_file = str3 + std::to_string(n) + image_file;

    std::vector<pointXYZRGB> lidar_points;
    cv::Mat image;
    lidar_points = readpointcloud(lidar_points_file);
    image = cv::imread(image_file);

    std::string calib_cam = "./camera/hk_right_params.yml";
    eigen_Cam_intrin = readCamintrin(calib_cam);

    double param[6];
    cal_angleAxis_from_matrix(eigen_T, param);
    std::cout << "x: " << param[0] << " y: " << param[1] << " z: " << param[2] << " d1: " << param[3] << " d2: " << param[4] << " d3: "<< param[5] << std::endl;
    std::vector<pointXYZRGB> temp1;
    for (int i = 0; i < lidar_points.size(); i++)
    {   
    double x = param[0]; double y = param[1]; double z = param[1];
    //std::cout << "x: " << x << " y: " << y << " z: " << z << " d1: " << param[3] << " d2: " << param[4] << " d3: "<< param[5] << std::endl;
    double pt[3], result[3];

    result[0] = cos(z)*cos(y)*pt[0] + (sin(z)*cos(x) + sin(x)*cos(z)*sin(y))*pt[1] + (sin(z)*sin(x) - cos(z)*sin(y)*cos(x))*pt[2];
    result[1] = -sin(z)*cos(y)*pt[0] + (cos(z)*cos(x) - sin(x)*sin(y)*sin(z))*pt[1] + (cos(z)*sin(x) + cos(x)*sin(z)*sin(y))*pt[2];
    result[2] = sin(y)*pt[0] -(sin(x)*cos(y))*pt[1] + (cos(x)*cos(y))*pt[2];
    //std::cout << result[2] << std::endl;
    if (result[2] > 0)
        {
        result[0] += param[3]; result[1] += param[4]; result[2] += param[5];
                
        double pu_cam1 = result[0] / result[2];
        double pv_cam1 = result[0] / result[2];

        double pu = eigen_Cam_intrin(0,0) * pu_cam1 + eigen_Cam_intrin(0,2);
        double pv = eigen_Cam_intrin(1,1) * pv_cam1 + eigen_Cam_intrin(1,2);

        std::cout << pu << " " << pv << std::endl;
        lidar_points[i]._uvz(0,0) = pu;
        lidar_points[i]._uvz(1,0) = pv;
        lidar_points[i]._uvz(2,0) = result[2];
        temp1.push_back(lidar_points[i]);
        }
    }

    lidar_points = temp1;

    //std::cout << "eigen_Cam_intrin: \n" << eigen_Cam_intrin << std::endl;
    //savepointcloud(lidar_points);
    fusion(lidar_points, image);

    namedWindow("image", WINDOW_NORMAL);
    imshow("image", image);
    waitKey(0);
}

int main()
{
    
    test();
    return 0;
}