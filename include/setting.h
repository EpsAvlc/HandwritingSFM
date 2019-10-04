/*
 * Created on Tue Sep 24 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef SETTING_H__
#define SETTING_H__

#include <iostream>
#include <vector>

// #include <Eigen/Core> 
#include <opencv2/core/core.hpp>

class Setting
{
public:
    Setting(const std::string& config_path);
    std::vector<std::string> imagePaths() {return image_strs_;};
    cv::Mat& K() {return K_;};
private:
    std::vector<std::string> image_strs_;
    cv::Mat K_;    
};
#endif