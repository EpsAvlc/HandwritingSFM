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
#include <thread>

#include <opencv2/core/core.hpp>

#include "frame.h"
#include "mappoint.h"
#include "viewer.h"

class HWSFM
{
public:
    HWSFM();

    void AddImages(cv::Mat& img);
    void SetCameraIntrins(cv::Mat& K);
    const std::vector<MapPoint> GetMappoints() {return mappoints_;};
    void StartReconstruction();
private:

    /**
     * @brief Use first two images to init scale.
     * 
     */
    void initScale();

    /**
     * @brief Use flannbasedMatcher to match keypoins
     * 
     * @param lhs Frame left hand side.
     * @param rhs Frame right hand side.
     * @param good_matches [output] vector to store good matches. 
     */
    void matchFeatures(Frame& lhs, Frame& rhs, std::vector<cv::DMatch>& good_matches);

    /**
     * @brief Triangulate points got from matching features.
     * 
     * @param l_index lhs frames index in class parameters frames_; 
     * @param r_index lhs frames index in class parameters frames_;
     * @param good_matches vector to store good matches. 
     */
    void triangulation(int l_index, int r_index, const std::vector<cv::DMatch>& good_matches);

    /**
     * @brief Convert pixel point into camera coordinate point.
     * 
     * @param pixel_pt pixel point
     * @return cv::Point3f camera point with z = 1;
     */
    cv::Point2f pixel2Camera(const cv::Point2f& pixel_pt);

    /**
     * @brief Reduce vector vec by status (often used in RANSAC) 
     * 
     * @tparam T 
     * @param vec vector to be reduced 
     * @param status vector computed by RANSAC version of findEsenstialMat. 
     */
    template <typename T> 
    void reduceVector(std::vector<T>& vec, std::vector<uchar> status)
    {
        int index = 0;
        for(int i = 0; i < vec.size(); i++)
        {
            if(status[i] == 1)
            {
                vec[index] = vec[i];
                index ++;             
            }
        }
        vec.resize(index);
    }

    /* private parameters */
    cv::Mat K_;
    std::vector<Frame> frames_;
    std::vector<std::vector<cv::KeyPoint>> features_;
    std::vector<std::vector<cv::Mat>> descriptors_;
    std::vector<MapPoint> mappoints_;
    Viewer viewer_;    
    std::thread viewer_thread_;
};

#endif