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
    vector<Mat> bgr;
    split(img, bgr);
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    vector<Mat> bgr_eq(3);
    clahe->apply(bgr[0], bgr_eq[0]);
    clahe->apply(bgr[1], bgr_eq[1]);
    clahe->apply(bgr[2], bgr_eq[2]);

    // for(int i = 0; i < 3; i++)
    // {
    //     equalizeHist(bgr[i], bgr_eq[i]);
    // }
    Mat img_eq;
    merge(bgr_eq, img_eq);

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