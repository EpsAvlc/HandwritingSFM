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
    cout << "----------SETTING INFORMATION----------" << endl;
    FileStorage fs(config_path, FileStorage::READ);
    string data_path;
    fs["data_path"] >> data_path;
    cout << "Dataset path is : " << data_path << endl;    
    vector<string> imgs_path;
    fs["image_list"] >> imgs_path;
    for(int i = 0; i < imgs_path.size(); i++)
    {
        image_strs_.push_back(data_path+ "/image data/" + imgs_path[i]);
    }
    cout << "--------------------------------------" << endl;
}