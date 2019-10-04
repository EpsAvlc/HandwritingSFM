/*
 * Created on Tue Sep 24 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "setting.h"

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

Setting::Setting(const string& config_path)
{

    FileStorage fs(config_path, FileStorage::READ);
    string data_path;
    fs["data_path"] >> data_path;
    vector<string> imgs_path;
    fs["image_list"] >> imgs_path;
    for(int i = 0; i < imgs_path.size(); i++)
    {
        image_strs_.push_back(data_path + "/" + imgs_path[i]);
    }
    float fx, fy, cx, cy;
    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["cx"] >> cx;
    fs["cy"] >> cy;
    K_ = Mat(3, 3, CV_32FC1, Scalar(0));
    K_.at<float>(0, 0) = fx;
    K_.at<float>(1, 1) = fy;
    K_.at<float>(0, 2) = cx;
    K_.at<float>(1, 2) = cy;
    K_.at<float>(2, 2) = 1;

    cout << "----------SETTING INFORMATION----------" << endl;
    cout << "Dataset path : " << data_path << endl;    
    cout << "Camera intrins : " << endl << K_ << endl;
    cout << "--------------------------------------" << endl;
}