#include "cluster.h"

float abs_value(float x)
{
    if(x < 0)
    return -x;
}

std::vector<pointXYZRGB> clustering_1(std::vector<pointXYZRGB>& lidar_points, float& w0, float w1, float w2)
{
    int times = 0;
    std::vector<pointXYZRGB> clustered_lidar_points;
    while (true)
    {
        float w0_old = w0; float w1_old = w1; float w2_old = w2;
        int n0 = 0; int n1 = 0; int n2 = 0;
        float sum0 = 0; float sum1 = 0; float sum2 = 0;
        times++;
        for (int i = 0; i < lidar_points.size(); i++)
        {
            if( (lidar_points[i]._reflection_rate - w0)*(lidar_points[i]._reflection_rate - w0) < (lidar_points[i]._reflection_rate - w1)*(lidar_points[i]._reflection_rate - w1)
            && (lidar_points[i]._reflection_rate - w0)*(lidar_points[i]._reflection_rate - w0) < (lidar_points[i]._reflection_rate - w2)*(lidar_points[i]._reflection_rate - w2) )
            {
            lidar_points[i].flag = 0;
            sum0 += lidar_points[i]._reflection_rate;
            n0++;
            }
        
            else if 
            ( (lidar_points[i]._reflection_rate - w1)*(lidar_points[i]._reflection_rate - w1) < (lidar_points[i]._reflection_rate - w0)*(lidar_points[i]._reflection_rate - w0)
            && (lidar_points[i]._reflection_rate - w1)*(lidar_points[i]._reflection_rate - w1) < (lidar_points[i]._reflection_rate - w2)*(lidar_points[i]._reflection_rate - w2) )
            {
                lidar_points[i].flag = 1;
                sum1 += lidar_points[i]._reflection_rate;
                n1++;
            }
            else if 
            ( (lidar_points[i]._reflection_rate - w2)*(lidar_points[i]._reflection_rate - w2) <= (lidar_points[i]._reflection_rate - w1)*(lidar_points[i]._reflection_rate - w1)
            && (lidar_points[i]._reflection_rate - w2)*(lidar_points[i]._reflection_rate - w2) <= (lidar_points[i]._reflection_rate - w0)*(lidar_points[i]._reflection_rate - w0) )
            {
                lidar_points[i].flag = 2;
                sum2 += lidar_points[i]._reflection_rate;
                n2++;
            }
        }
    //std::cout << "Iteration times: " << times << std::endl;
    //std::cout << n0 + n1 + n2 << std::endl;
    w0 = sum0 / n0;
    w1 = sum1 / n1;
    w2 = sum2 / n2;
    //std::cout << w0 << " " << w1 << " " << w2 << std::endl;

    if( abs_value(w0 - w0_old) < 1 && abs_value(w1 - w1_old) < 1 && abs_value(w2 - w2_old) < 1)
    {
        
        if (w1 < w0 && w2 < w0)
        {
            for (int i = 0; i < lidar_points.size(); i++)
            {
                if (lidar_points[i].flag == 0)
                {
                    clustered_lidar_points.push_back(lidar_points[i]);
                }
            }           
        }
        else if (w0 < w1 && w2 < w1)
        {
            for (int i = 0; i < lidar_points.size(); i++)
            {
                if (lidar_points[i].flag == 1)
                {
                    lidar_points[i].flag = 0;
                    clustered_lidar_points.push_back(lidar_points[i]);
                }
            }    
        }
        else
        {
            for (int i = 0; i < lidar_points.size(); i++)
            {
                if (lidar_points[i].flag == 2)
                {
                    lidar_points[i].flag = 0;
                    clustered_lidar_points.push_back(lidar_points[i]);
                }
            }    
        }
        
        break;
    }
    }

    return clustered_lidar_points;
}

double distance(pointXYZRGB& point1, pointXYZRGB& point2)
{
    double x2 = (point1._point(0,0)-point2._point(0,0)) * (point1._point(0,0)-point2._point(0,0));
    double y2 = (point1._point(1,0)-point2._point(1,0)) * (point1._point(1,0)-point2._point(1,0));
    double z2 = (point1._point(2,0)-point2._point(2,0)) * (point2._point(2,0)-point2._point(2,0));
    return sqrt(x2 + y2 + z2);
}

double height_distance(pointXYZRGB& point1, pointXYZRGB& point2)
{
    double z2 = (point1._point(2,0) - point2._point(2,0)) * (point1._point(2,0) - point2._point(2,0));
    return sqrt(z2);
}

void meanshift(pointXYZRGB center_point, std::vector<pointXYZRGB>& lidar_points,int& flag) 
{
    pointXYZRGB centerpoint_old;
    Eigen::Matrix<double, 3, 1> meanshift;
        
    while (true)
    {
    int a = 0;
    centerpoint_old = center_point;
    meanshift << 0, 0, 0;

    for (int i = 0; i < lidar_points.size(); i++)
    {
        if (distance(lidar_points[i], center_point) <= 0.8 && lidar_points[i].flag == 0)
        {          
        a++;
        meanshift += lidar_points[i]._point - center_point._point;
        }
    }
        
    //std::cout << "a： " << a << std::endl;
    //std::cout << "meanshift: \n" << meanshift << std::endl;
    //std::cout << "displayed_points: " << displayed_points.size() << std::endl;
    center_point._point = centerpoint_old._point + meanshift/a;

    if(distance(center_point, centerpoint_old) < 0.02)
    {
        for (int i = 0; i < lidar_points.size(); i++)
        {
            if (distance(lidar_points[i], center_point) <= 0.8 && lidar_points[i].flag == 0)
            {
                lidar_points[i].flag = flag;
            }
        }
        flag++;
        break;
    }
    }
}

void clustering_line(std::vector<pointXYZRGB>& lidar_points, int flag, std::vector<pointXYZRGB>& circle_points)
{
    int flag_line = flag + 1;
    bool FLAG = true;
    while (FLAG)
    {
        FLAG = false;
        for (int j = 0; j < lidar_points.size(); j++)
        {
            if (lidar_points[j].flag == flag)
            {
                for (int i = 0; i < lidar_points.size(); i++)
                {
                    if (height_distance(lidar_points[i], lidar_points[j]) < 0.1 && lidar_points[i].flag == flag)
                    {
                        lidar_points[i].flag = flag_line;
                    } 
                }
                flag_line++;
                FLAG = true;
                break;
            }
        }
    }

    
    for (int i = flag+1; i < flag_line; i++)
    {
       std::vector<pointXYZRGB> temp;
       for (int j = 0; j < lidar_points.size(); j++)
       {
           if (lidar_points[j].flag == i)
           {
               temp.push_back(lidar_points[j]);
           }
       }
       pointXYZRGB y_min, y_max;
       y_min = y_max = temp[0];

       for (int k = 0; k < temp.size(); k++)
       {
           if (temp[k]._point(1,0) < y_min._point(1,0)) y_min = temp[k];
           if (temp[k]._point(1,0) > y_max._point(1,0)) y_max = temp[k];  
       }
        circle_points.push_back(y_max);
        circle_points.push_back(y_min);
    }
}

void extracting_circle_center(std::vector<pointXYZRGB>& lidar_points, int& classes, std::vector<pointXYZRGB>& circle_points)
{
    std::vector<std::vector<pointXYZRGB>> A;
    std::vector<pointXYZRGB> temp;
    for (int i = 1; i < classes+1; i++)
    {
        for (int j = 0; j < lidar_points.size(); j++)
        {
            if (lidar_points[j].flag == i )
            {
                temp.push_back(lidar_points[j]);
            }
        }
        A.push_back(temp);
        temp.clear();
    }
    
    for (int i = 0; i < classes; i++)
    {
        clustering_line(A[i],A[i][0].flag, circle_points);  
    }

    for (int i = 0; i < circle_points.size(); i++)
    {
        circle_points[i].flag = 0;
    }

    lidar_points = circle_points;
}  

void clustering_2(std::vector<pointXYZRGB>& lidar_points, int& classes)
{
    int flag = 1;
    
    while (true)
    {   
        bool FLAG = true;
        for (int i = 0; i < lidar_points.size(); i++)
        {
            if (lidar_points[i].flag == 0)
            {    
                meanshift(lidar_points[i], lidar_points, flag);
                FLAG = false;
                break; 
            }
        }
        
        if (FLAG)
        {
            classes = flag-1 ;
            //std::cout << classes << " classes." << std::endl;
            return;
        }
    }
}

Eigen::Matrix<double, 3, 1> x ( Eigen::Matrix<double,3,1>& point1, Eigen::Matrix<double,3,1>& point2)
{
    Eigen::Matrix<double, 3, 1> p1, p2, p;
    p1 << point1(0,0), point1(1,0), point1(2,0);
    p2 << point2(0,0), point2(1,0), point2(2,0);

    double i = p1(1,0) * p2(2,0) - p1(2,0) * p2(1,0);
    double j = p1(2,0) * p2(0,0) - p1(0,0) * p2(2,0);
    double k = p1(0,0) * p2(1,0) - p1(1,0) * p2(0,0);
    p << i, j, k;
    return p;
}

double dot(Eigen::Matrix<double, 3, 1>& vector1, Eigen::Matrix<double, 3, 1>& vector2)
{
    return vector1(0,0) * vector2(0,0) + vector1(1,0) * vector2(1,0) + vector1(2,0) * vector2(2,0);
}

double norm(Eigen::Matrix<double, 3, 1>& vector)
{
    return sqrt(vector(0,0)*vector(0,0) + vector(1,0)*vector(1,0) + vector(2,0)*vector(2,0));
}

Eigen::Matrix<double, 3, 1> ceres_solve_plane(std::vector<pointXYZRGB>& lidar_points, pointXYZRGB& center_point)
{
    double abc[3] = {center_point._point(0,0), center_point._point(1,0),center_point._point(2,0)};
    ceres::Problem problem;
    for (int i = 0; i < lidar_points.size(); i++)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<FITTING_PLANE_COST,1,3>(
                new FITTING_PLANE_COST(lidar_points[i]._point(0,0), lidar_points[i]._point(1,0), lidar_points[i]._point(2,0), center_point)
            ),
            nullptr,
            abc
        );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << "a= " << abc[0] << std::endl;
    //std::cout << "b= " << abc[1] << std::endl;
    //std::cout << "c= " << abc[2] << std::endl;

    Eigen::Matrix<double, 3, 1> normal_vector;
    normal_vector << abc[0], abc[1], abc[2];
    return normal_vector;
}

Eigen::Matrix<double, 3, 1> ceres_solve_circlecenter(std::vector<pointXYZRGB>& temp_points, pointXYZRGB& center_point)
{
    double abr[3] = {center_point._point(0,0), center_point._point(1,0), 0.4};
    ceres::Problem problem;
    for (int i = 0; i < temp_points.size(); i++)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<FITTING_CIRCLRCENTER_COST, 1, 3>(
                new FITTING_CIRCLRCENTER_COST(temp_points[i]._point(0,0), temp_points[i]._point(1,0))
            ),
            nullptr,
            abr
        );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << "a= " << abr[0] << std::endl;
    //std::cout << "b= " << abr[1] << std::endl;
    //std::cout << "r= " << abr[2] << std::endl;

    Eigen::Matrix<double, 3, 1> circle;
    circle << abr[0], abr[1], 0;
    return circle;
}

pointXYZRGB extracting_circle(std::vector<pointXYZRGB>& lidar_points)
{
    pointXYZRGB A, centerpoint;
    A._point << 0, 0, 0;
    for (int i = 0; i < lidar_points.size(); i++)
    {
        A._point += lidar_points[i]._point;
    }
    centerpoint._point << A._point(0,0)/lidar_points.size(), A._point(1,0)/lidar_points.size(), A._point(2,0)/lidar_points.size();
    
    //std::cout << "centerpoint: " << centerpoint._point << std::endl;
    
    Eigen::Matrix<double, 3, 1> normal_vector, x_vector, y_vector;
    
    normal_vector = ceres_solve_plane(lidar_points, centerpoint);
    /**
    for (int i = 0; i < lidar_points.size(); i++)
    {
    Eigen::Matrix<double, 3, 1> vector;
    vector << lidar_points[i]._point(0,0)-centerpoint._point(0,0),
              lidar_points[i]._point(1,0)-centerpoint._point(1,0),
              lidar_points[i]._point(2,0)-centerpoint._point(2,0);
    std::cout << "normal_vector: " << dot(normal_vector, vector)/(norm(normal_vector)*norm(vector)) << std::endl;
    }
    **/
    x_vector << 1 , 1, (-normal_vector(0,0)-normal_vector(1,0))/normal_vector(2,0);
    y_vector = x(x_vector, normal_vector);
    
    //std::cout << dot(normal_vector, x_vector) << " " << dot(normal_vector, y_vector) << " " << dot(y_vector, x_vector)  << std::endl;
    
    Eigen::Matrix<double, 3, 1> frame_x, frame_y, frame_z;

    frame_x << x_vector(0,0)/norm(x_vector), x_vector(1,0)/norm(x_vector), x_vector(2,0)/norm(x_vector);
    frame_y << y_vector(0,0)/norm(y_vector), y_vector(1,0)/norm(y_vector), y_vector(2,0)/norm(y_vector);
    frame_z << normal_vector(0,0)/norm(normal_vector), normal_vector(1,0)/norm(normal_vector), normal_vector(2,0)/norm(normal_vector);

    //std::cout << frame_x << "\n" << frame_y << "\n" << frame_z << std::endl;
    //std::cout << dot(frame_x,frame_y) << dot(frame_x, frame_z) << dot(frame_y, frame_z) << std::endl; 

    Eigen::Matrix3d R_12;
    Eigen::Matrix<double, 3, 1> x, y, z;
    x << 1, 0, 0;
    y << 0, 1, 0;
    z << 0 ,0, 1;

    R_12 << dot(x, frame_x), dot(y, frame_x), dot(z, frame_x),
            dot(x, frame_y), dot(y, frame_y), dot(z, frame_y),
            dot(x, frame_z), dot(y, frame_z), dot(z, frame_z);

    Eigen::Matrix<double, 3, 1> position;
    position = R_12 * (-centerpoint._point);

    Eigen::Matrix4d T_12;

    T_12 << R_12(0,0), R_12(0,1), R_12(0,2), position(0,0),
            R_12(1,0), R_12(1,1), R_12(1,2), position(1,0),
            R_12(2,0), R_12(2,1), R_12(2,2), position(2,0),
                    0,         0,         0,             1;


    //std::cout << T_12 << std::endl;
    //std::cout << dot(x_vector,y_vector) << " " << dot(x_vector,normal_vector) << " " << dot(y_vector, normal_vector) << std::endl;
    std::vector<pointXYZRGB> temp_points;
    pointXYZRGB temp_point;
    Eigen::Matrix<double, 4, 1> temp_point_homogeneous;

    for (int i = 0; i < lidar_points.size(); i++)
    {
        temp_point_homogeneous << lidar_points[i]._point(0,0), lidar_points[i]._point(1,0), lidar_points[i]._point(2,0), 1;
        temp_point._point << (T_12 * temp_point_homogeneous)(0,0), (T_12 * temp_point_homogeneous)(1,0), (T_12 * temp_point_homogeneous)(2,0);
        temp_point.flag = 3;
        temp_points.push_back(temp_point);
    }

    //savepointcloud(temp_points);
    
    Eigen::Matrix<double, 3, 1> circlecenter;

    circlecenter =  ceres_solve_circlecenter(temp_points, centerpoint);
    pointXYZRGB B;
    B._point = circlecenter; B.flag=6;
    temp_points.push_back(B);
    
    Eigen::Matrix<double, 4, 1> circlecenter_homogeneous;
    circlecenter_homogeneous << circlecenter(0,0), circlecenter(1,0), circlecenter(2,0), 1; 
    pointXYZRGB circlecenter_point;
    circlecenter_point._point << (T_12.inverse()*circlecenter_homogeneous)(0,0), (T_12.inverse()*circlecenter_homogeneous)(1,0), (T_12.inverse()*circlecenter_homogeneous)(2,0);
    circlecenter_point.flag = lidar_points[0].flag;
    return circlecenter_point;
    //lidar_points.push_back(circlecenter_point);
    //savepointcloud(lidar_points);
}

std::vector<pointXYZRGB> extracting_circle_center(std::vector<pointXYZRGB>& lidar_points, int& classes)
{
    
    std::vector<std::vector<pointXYZRGB>> AA;
    std::vector<pointXYZRGB> temp, lidar_center_points;

    for (int i = 1; i < classes+1; i++)
    {
        for (int j = 0; j < lidar_points.size(); j++)
        {
            if (lidar_points[j].flag == i)
            {
                temp.push_back(lidar_points[j]);
            }
        }
        AA.push_back(temp);
        temp.clear();
    }
    
    pointXYZRGB center_point;
    for (int i = 0; i < classes; i++)
    {
        center_point = extracting_circle(AA[i]);
        center_point._reflection_rate = 255;
        lidar_center_points.push_back(center_point);
    }

    return lidar_center_points;
}

double uv_distance(pointXYZRGB& lidar_center_point, Eigen::Matrix<double, 2, 1>& image_center_point)
{
    double u2 = (lidar_center_point._uvz(0,0)-image_center_point(0,0))*(lidar_center_point._uvz(0,0)-image_center_point(0,0));
    double v2 = (lidar_center_point._uvz(1,0)-image_center_point(1,0))*(lidar_center_point._uvz(1,0)-image_center_point(1,0));
    double distance = sqrt(u2 + v2);
    return distance;
}

std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>> pair_center_point(pointXYZRGB& lidar_center_point, std::vector<Eigen::Matrix<double, 2, 1>>& image_center_point)
{
    std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>> paired_center_point;
    Eigen::Matrix<double, 2, 1> pair_point;
    pair_point = image_center_point[0];
    for (int i = 1; i < image_center_point.size(); i++)
    {
        if (uv_distance(lidar_center_point, image_center_point[i]) < uv_distance(lidar_center_point, pair_point))
        {
            pair_point = image_center_point[i];
        }
    }
    paired_center_point = {lidar_center_point, pair_point};
    return paired_center_point;
}

void pair_center_points(std::vector<pointXYZRGB>& lidar_center_points, std::vector<Eigen::Matrix<double, 2, 1>>& image_center_points, 
std::vector<std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>>>& paired_center_points)
{
    //std::cout << lidar_center_points.size() << std::endl;
    std::pair<pointXYZRGB, Eigen::Matrix<double, 2, 1>> paired_center_point;
    for (int i = 0; i < lidar_center_points.size(); i++)
    {
        paired_center_point = pair_center_point(lidar_center_points[i], image_center_points);
        if (uv_distance(paired_center_point.first, paired_center_point.second) <= 50)
        {
            paired_center_points.push_back(paired_center_point);
        }
    }
}