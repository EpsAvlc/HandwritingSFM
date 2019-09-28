/*
 * Created on Tue Sep 24 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef HWSFM_H__
#define HWSFM_H__
#include <iostream>

#include <opencv2/core/core.hpp>


#include "frame.h"

class HWSFM
{
public:
    HWSFM(){};
    void AddImages(cv::Mat& img);
    void StartReconstruction();
private:
    void initScale();
    void matchFeatures(Frame& lhs, Frame& rhs, std::vector<cv::DMatch>& good_matches);
    std::vector<Frame> frames_;
    std::vector<std::vector<cv::KeyPoint>> features_;
    std::vector<std::vector<cv::Mat>> descriptors_;
};

#endif