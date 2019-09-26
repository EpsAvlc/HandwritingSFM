/*
 * Created on Thu Sep 26 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef IMAGE_H__
#define IMAGE_H__

#include <iostream>
#include <opencv2/core/core.hpp>

class Image
{
public:
    Image(cv::Mat& img) 
    {
        id_ ++;
        extractFeatures(img);
    }
    int Id() { return id_;};

private:
    void extractFeatures(cv::Mat& img);
    static int id_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descritor_;
};

#endif 