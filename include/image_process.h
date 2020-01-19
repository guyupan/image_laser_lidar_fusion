#ifndef IMAGE_PROCESS
#define IMAGE_PROCESS

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "pointXYZRGB.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/eigen.hpp>
using namespace cv;

void show_B_histogram(Mat& source_image)
{
    int channels = 0;
    int histsize[] = {256};
    float midranges[] = {0, 255};
    const float *ranges[] = { midranges };
    MatND B_histogram;
    
    calcHist(&source_image, 1, &channels, Mat(), B_histogram, 1, histsize, ranges, true, false);
    Mat b_drawImage = Mat::zeros(Size(256,256), CV_8UC3);

    double b_dhistmaxvalue;
    Point b_mostvalue;
    minMaxLoc(B_histogram, 0, &b_dhistmaxvalue, &b_mostvalue, 0);
    std::cout << b_mostvalue << std::endl;
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(256 * 0.9 * (B_histogram.at<float>(i)/b_dhistmaxvalue));
        line(b_drawImage, Point(i, b_drawImage.rows - 1), Point(i, b_drawImage.rows - 1 - value), Scalar(255, 0, 0));
    }
    namedWindow("B_channel hist", WINDOW_NORMAL);
    imshow("B_channel hist", b_drawImage);
}

void show_G_histogram(Mat& source_image)
{
    int channels = 1;
    int histsize[] = {256};
    float midranges[] = {0, 255};
    const float *ranges[] = { midranges };
    MatND G_histogram;
    
    calcHist(&source_image, 1, &channels, Mat(), G_histogram, 1, histsize, ranges, true, false);
    Mat g_drawImage = Mat::zeros(Size(256,256), CV_8UC3);

    double g_dhistmaxvalue;
    Point g_mostvalue;
    minMaxLoc(G_histogram, 0, &g_dhistmaxvalue, &g_mostvalue, 0);
    std::cout << g_mostvalue << std::endl;
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(256 * 0.9 * (G_histogram.at<float>(i)/g_dhistmaxvalue));
        line(g_drawImage, Point(i, g_drawImage.rows - 1), Point(i, g_drawImage.rows - 1 - value), Scalar(255, 0, 0));
    }
    namedWindow("G_channel hist", WINDOW_NORMAL);
    imshow("G_channel hist", g_drawImage);
}

void show_H_histogram(Mat& source_image)
{
    Mat srcimgHSV ;
    cvtColor(source_image, srcimgHSV, COLOR_BGR2HSV);

    namedWindow("hsv_image", WINDOW_NORMAL);
    imshow("hsv_image", srcimgHSV);

    int channels = 0;
    int histsize[] = {180};
    float midranges[] = {0, 180};
    const float *ranges[] = { midranges };
    MatND H_histogram;
    
    calcHist(&srcimgHSV, 1, &channels, Mat(), H_histogram, 1, histsize, ranges, true, false);
    Mat h_drawImage = Mat::zeros(Size(180,180), CV_8UC3);

    double h_dhistmaxvalue;
    Point h_mostvalue;
    minMaxLoc(H_histogram, 0, &h_dhistmaxvalue, &h_mostvalue, 0);
    std::cout << h_mostvalue << std::endl;
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(256 * 0.9 * (H_histogram.at<float>(i)/h_dhistmaxvalue));
        line(h_drawImage, Point(i, h_drawImage.rows - 1), Point(i, h_drawImage.rows - 1 - value), Scalar(255, 0, 0));
    }
    namedWindow("H_channel hist", WINDOW_NORMAL);
    imshow("H_channel hist", h_drawImage);
}

void show_V_histogram(Mat& source_image)
{
    Mat srcimgHSV ;
    cvtColor(source_image, srcimgHSV, COLOR_BGR2HSV);

    int channels = 2;
    int histsize[] = {256};
    float midranges[] = {0, 255};
    const float *ranges[] = { midranges };
    MatND V_histogram;
    
    calcHist(&srcimgHSV, 1, &channels, Mat(), V_histogram, 1, histsize, ranges, true, false);
    Mat v_drawImage = Mat::zeros(Size(256,256), CV_8UC3);

    double v_dhistmaxvalue;
    Point v_mostvalue;
    minMaxLoc(V_histogram, 0, &v_dhistmaxvalue, &v_mostvalue, 0);
    std::cout << v_mostvalue << std::endl;
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(256 * 0.9 * (V_histogram.at<float>(i)/v_dhistmaxvalue));
        line(v_drawImage, Point(i, v_drawImage.rows - 1), Point(i, v_drawImage.rows - 1 - value), Scalar(255, 0, 0));
    }
    namedWindow("V_channel hist", WINDOW_NORMAL);
    imshow("V_channel hist", v_drawImage);
}

void show_gray_image(Mat& source_image)
{
    Mat gray_image;
    cvtColor(source_image, gray_image, COLOR_BGRA2GRAY);
    namedWindow("gray_image", WINDOW_NORMAL);
    imshow("gray_image", gray_image);
}

Mat show_grayimage_b_channel(Mat& source_image)
{
    int rows = source_image.rows;
    int cols = source_image.cols;
    Mat b_image(rows, cols, CV_8UC1), b_image_blured(rows, cols, CV_8UC1);
    Mat labels, stats;

    Mat srcimgHSV ;
    cvtColor(source_image, srcimgHSV, COLOR_BGR2HSV);

    for (int i = 0; i < source_image.rows; i++)
    {
        for (int j = 0; j < source_image.cols; j++)
        {
            if ( 

            source_image.at<Vec3b>(i,j)[0] > source_image.at<Vec3b>(i,j)[1] 
            && source_image.at<Vec3b>(i,j)[0] > source_image.at<Vec3b>(i,j)[2]
        
            &&srcimgHSV.at<Vec3b>(i,j)[0] > 100 && srcimgHSV.at<Vec3b>(i,j)[0] < 150 
            /*
            && srcimgHSV.at<Vec3b>(i,j)[1] > 100 && srcimgHSV.at<Vec3b>(i,j)[1] < 255 
            && srcimgHSV.at<Vec3b>(i,j)[2] > 40 && srcimgHSV.at<Vec3b>(i,j)[2] < 255
            */ )
            {
                b_image.at<uchar>(i,j) = 255;
            } 
            else
            {
                b_image.at<uchar>(i,j) = 0;            }
            
        }
    }
  
    for (int i = 0; i < 1; i++)
    {

    Size2i kernel(3,3);
    blur(b_image, b_image_blured, kernel);
    threshold(b_image_blured, b_image, 128, 255, THRESH_BINARY);
    //namedWindow("b_image_blured", WINDOW_NORMAL);
    //imshow("b_image_blured", b_image_blured);
   
    }
    

    Mat centroids;
    int nccomps = connectedComponentsWithStats(b_image, labels, stats, centroids, 4);


    for (int y = 0; y < b_image.rows; y++)
    {
        for (int x = 0; x < b_image.cols; x++)
        {
            int label = labels.at<int>(y,x);
            if ( stats.at<int>(label, CC_STAT_AREA) < 40000)
            {
                b_image.at<uchar>(y,x) = 0;
            }
        }
    }
    
    //std::cout << nccomps << std::endl;

    //namedWindow("b_image", WINDOW_NORMAL);
    //imshow("b_image", b_image);

 

    return b_image; 
}

void show_hsvimage_v_channel(Mat& source_image)
{
    int rows = source_image.rows;
    int cols = source_image.cols;
    Mat gray_image(rows, cols, CV_8UC1);
    Mat hsv_img;
    cvtColor(source_image, hsv_img, COLOR_BGR2HSV);


    for (int i = 0; i < source_image.rows; i++)
    {
        for (int j = 0; j < source_image.cols; j++)
        {
            gray_image.at<uchar>(i,j) = source_image.at<Vec3b>(i,j)[2];
        }
    }
    namedWindow("gray_image_v_channel", WINDOW_NORMAL);
    imshow("gray_image_v_channel", gray_image);
}

void show_hsvimage_s_channel(Mat& source_image)
{
    int rows = source_image.rows;
    int cols = source_image.cols;
    Mat gray_image(rows, cols, CV_8UC1);
    Mat hsv_img;
    cvtColor(source_image, hsv_img, COLOR_BGR2HSV);


    for (int i = 0; i < source_image.rows; i++)
    {
        for (int j = 0; j < source_image.cols; j++)
        {
            gray_image.at<uchar>(i,j) = source_image.at<Vec3b>(i,j)[1];
        }
    }
    namedWindow("gray_image_s_channel", WINDOW_NORMAL);
    imshow("gray_image_s_channel", gray_image);
}

std::vector<Eigen::Matrix<double, 2, 1>> find_circle_centers(Mat& source_image)
{

    Eigen::Matrix<double, 2, 1> image_center_point;
    std::vector<Eigen::Matrix<double, 2, 1>> image_center_points;
    Mat binary_image = show_grayimage_b_channel(source_image);
    std::vector< std::vector<Point> > contours;
    findContours( binary_image, contours, noArray(), RETR_LIST, CHAIN_APPROX_SIMPLE );
    
    binary_image = Scalar::all(0);
    drawContours(binary_image, contours, -1, Scalar::all(255));

    for (size_t t = 0; t < contours.size(); t++) 
    {
        if (contours[t].size() > 100)
        {
        RotatedRect rrt = fitEllipse(contours[t]);
        ellipse(source_image, rrt, Scalar(0,0,255), 2);
        circle(source_image, rrt.center, 3.5, (0,0,0), -1);
        image_center_point << rrt.center.x, rrt.center.y;
        image_center_points.push_back(image_center_point);
        //std::cout << rrt.center << std::endl;
        }
    }


    //namedWindow("contour_image", WINDOW_NORMAL);
    //imshow("contour_image", binary_image);

    //namedWindow("result_image", WINDOW_NORMAL);
    //imshow("result_image", source_image);
    //waitKey(0);

    return image_center_points;
}



/*
int main()
{
    Mat srcimg = imread("000_L.png");
    //std::cout << "image's rows: " << srcimg.rows << " image's cols: " << srcimg.cols << " type: " << srcimg.type() <<  std::endl;
    //std::cout << int(srcimg.at<Vec3b>(0,0)[0]) << std::endl;
    namedWindow("source_image", WINDOW_NORMAL);
    imshow("source_image", srcimg);

    //show_B_histogram(srcimg);
    //show_G_histogram(srcimg);
    //show_V_histogram(srcimg);
    //show_gray_image(srcimg);
    //show_grayimage_b_channel(srcimg);  
    //show_hsvimage_v_channel(srcimg);
    //show_hsvimage_s_channel(srcimg);
    //find_contours(srcimg);
    //find_circle(srcimg);
    
    cv::waitKey(0);
    return 0;   
}
*/

#endif