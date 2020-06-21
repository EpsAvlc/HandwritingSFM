/*
 * Created on Thu Sep 26 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "frame.h"

#include <opencv2/xfeatures2d/nonfree.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int Frame::id_counter_ = 0;

void Frame::extractFeatures(const Mat& img)
{
    Mat img_gray;
    if(img.channels() == 3)
    {
        cvtColor(img, img_gray, COLOR_BGR2GRAY);
    }
    else
    {
        img_gray = img;
    }

    Mat img_eq;
    equalizeHist(img_gray, img_eq);

    // Mat img_eq = img.clone();

    Ptr<xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create();
    sift->detect(img_eq, keypoints_);
    sift->compute(img_eq, keypoints_, descriptor_);
}

bool Frame::AddTriangulated(int feature_index, int mappoint_id)
{
    if(triangulated_.count(feature_index) == 0)
    {
        triangulated_[feature_index] = mappoint_id;
        return true;
    }
    else
    {
        return false;
    }
}

int Frame::GetTriangulated(int feature_index)
{
    if(triangulated_.count(feature_index) == 0)
    {
        return -1;
    }
    else
    {
        return triangulated_[feature_index];
    }
    
}

void Frame::SetPose(g2o::SE3Quat& se3)
{
    Eigen::Quaterniond q = se3.rotation();
    Eigen::Matrix3d R_eigen = q.matrix();
    for(int i = 0; i < 3; i ++)
        for(int j = 0; j < 3; j++)
        {
            R_.at<float>(i, j) = R_eigen(i, j);
        }
    Eigen::Vector3d t_eigen = se3.translation();
    t_.at<float>(0, 0) = t_eigen.x();
    t_.at<float>(1, 0) = t_eigen.y();
    t_.at<float>(2, 0) = t_eigen.z();
}