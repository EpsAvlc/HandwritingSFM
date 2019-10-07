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
        // cout << image_strs_[i] << endl;
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

    fs["ViewerpointX"] >> viewer_point_x_;
    fs["ViewerpointY"] >> viewer_point_y_;
    fs["ViewerpointZ"] >> viewer_point_z_;
    fs["ViewerpointF"] >> viewer_point_f_;

    cout << "----------SETTING INFORMATION----------" << endl;
    cout << "Dataset path : " << data_path << endl;    
    cout << "Camera intrins : " << endl << K_ << endl;
    cout << "Viewerpoint x: " << viewer_point_x_ << endl;
    cout << "Viewerpoint y: " << viewer_point_y_ << endl;
    cout << "Viewerpoint z: " << viewer_point_z_ << endl;
    cout << "Viewerpoint f: " << viewer_point_f_ << endl;
    cout << "--------------------------------------" << endl;
}