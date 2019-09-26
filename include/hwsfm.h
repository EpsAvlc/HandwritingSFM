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

#include "image.h"

class HWSFM
{
public:
    HWSFM(){};
    void AddImages(cv::Mat& img);
    void StartReconstruction();
private:
    void extractFeatures();

    std::vector<Image> imgs_;
    std::vector<std::vector<cv::KeyPoint>> features_;
    std::vector<std::vector<cv::Mat>> descriptors_;
};

#endif