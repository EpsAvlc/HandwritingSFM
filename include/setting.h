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
    friend class HWSFM;
    friend class Viewer;
    Setting(const std::string& config_path);
private:
    std::vector<std::string> image_strs_;
    cv::Mat K_;    
    float viewer_point_x_, viewer_point_y_, viewer_point_z_, viewer_point_f_;
};
#endif