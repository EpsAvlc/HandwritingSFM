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
#include <unordered_map>

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
    bool AddTriangulated(int feature_index, int mappoint_id);
private:
    void extractFeatures(const cv::Mat& img);
    static int id_counter_;
    int id_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptor_;
    cv::Mat img_;
    cv::Mat R_;
    cv::Mat t_;
    /**
     * @brief map that stores the feature index that has been triangulated.
     * 
     */
    std::unordered_map<int, int> triangulated_;
};

#endif 