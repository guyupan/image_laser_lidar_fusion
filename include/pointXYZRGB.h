#ifndef POINTXYZRGB
#define POINTXYZRGB

#include <eigen3/Eigen/Dense>

struct pointXYZRGB
{
    Eigen::Matrix<double, 3, 1> _point;
    double _reflection_rate;
    Eigen::Matrix<double, 3, 1> _uvz;
    int flag = 0;
};

#endif
