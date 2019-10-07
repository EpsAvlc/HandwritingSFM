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

using namespace cv;
using namespace std;

int Frame::id_counter_ = 0;

void Frame::extractFeatures(const Mat& img)
{
    Ptr<xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create();
    sift->detect(img, keypoints_);
    sift->compute(img, keypoints_, descriptor_);
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