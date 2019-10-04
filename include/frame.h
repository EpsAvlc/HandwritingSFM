/*
 * Created on Thu Sep 26 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef FRAME_H__
#define FRAME_H__

#include <iostream>
#include <opencv2/core/core.hpp>

class Frame
{
public:
    Frame(const cv::Mat& img) 
    {
        id_ ++;
        img_ = img.clone();
        extractFeatures(img);
    }
    int Id() { return id_;};
    cv::Mat& Descriptor(){ return descriptor_;};
    const std::vector<cv::KeyPoint>& Keypoints() {return keypoints_;};
    const cv::Mat& Img() {return img_;};
    const cv::Mat& GetR() {return R_;};
    void SetR(cv::Mat R) {R_ = R;};
    const cv::Mat& GetT() {return t_;};
    void SetT(cv::Mat t) {t_ = t;};

private:
    void extractFeatures(const cv::Mat& img);
    static int id_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptor_;
    cv::Mat img_;
    cv::Mat R_;
    cv::Mat t_;
};

#endif 