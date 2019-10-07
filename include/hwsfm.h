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
#include <mutex>

#include <opencv2/core/core.hpp>

#include "frame.h"
#include "mappoint.h"
#include "viewer.h"
#include "setting.h"

class HWSFM
{
public:
    friend class Viewer;
    HWSFM(Setting& s);
    const std::vector<MapPoint> GetMappoints() {return mappoints_;};
    void StartReconstruction();
    // void SetViewerBusy(bool isbusy);
private:

    void addImages(cv::Mat& img);

    /**
     * @brief Use first two images to init scale.
     * 
     */
    void initScale();

    /**
     * @brief When inited scale, use pnp to solve R, t and triangulate
     *          points. 
     * 
     * @param lhs Frame left hand side that must have 
     *          triangulated points.
     * @param rhs Frame right hand side.
     */
    void solvePnPAndTriangulation(Frame& lhs, Frame& rhs);
    /**
     * @brief Use flannbasedMatcher to match keypoins
     * 
     * @param lhs Frame left hand side.
     * @param rhs Frame right hand side.
     * @param good_matches [output] vector to store good matches. 
     */
    void matchFeatures(Frame& lhs, Frame& rhs, std::vector<cv::DMatch>& good_matches);

    /**
     * @brief use ransac method of find fundamental to remove bad matches;
     * 
     * @param lhs Frame left hand side .
     * @param rhs Frame right hand side.
     * @param matches initial matches;
     */
    void rejectWithF(Frame& lhs, Frame& rhs, std::vector<cv::DMatch>& matches);
    /**
     * @brief Triangulate points got from matching features.
     * 
     * @param lhs lhs frames; 
     * @param rhs rhs frames;
     * @param good_matches vector to store good matches. 
     */
    void triangulation(Frame& lhs, Frame& rhs, const std::vector<cv::DMatch>& good_matches);

    /**
     * @brief Convert pixel point into camera coordinate point.
     * 
     * @param pixel_pt pixel point
     * @return cv::Point3f camera point with z = 1;
     */
    cv::Point2f pixel2Camera(const cv::Point2f& pixel_pt);

    cv::Point2f camera2Pixel(const cv::Point2f& cam_pt);
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
    Setting& setting_;
    std::thread viewer_thread_;
    std::mutex viewer_mutex_;
};

#endif