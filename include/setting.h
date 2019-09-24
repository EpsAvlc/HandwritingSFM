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

class Setting
{
public:
    Setting(const std::string& config_path);
    std::vector<std::string> imagePaths() {return image_strs_;};
private:
    std::vector<std::string> image_strs_;
};
#endif