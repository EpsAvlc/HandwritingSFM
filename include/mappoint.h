/*
 * Created on Wed Sep 25 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef MAPPOINT_H__
#define MAPPOINT_H__

#include <iostream>
#include <unordered_map>

#include <opencv2/core/core.hpp>

#include "frame.h"

class MapPoint
{
public:
    MapPoint(){id_ = id_counter_; id_counter_ ++; };
    MapPoint(double x, double y, double z) : x_(x), y_(y), z_(z)
    {
        id_ = id_counter_; 
        id_counter_++;
    };
    int Id() {return id_;};
    bool AddObserver(Frame& frame, int feature_index);
    int QueryObserver(int frame_index);
    std::unordered_map<int, int> GetObservers(){return observers_;};
    float& x(){return x_;};
    float& y(){return y_;};
    float& z(){return z_;};
    cv::Point3f GetWorldPos() const { return cv::Point3f(x_, y_, z_);};
    cv::Scalar GetColor() const {return color_;};
    void SetColor(cv::Scalar color) { color_ = color; };
    
private:
    void updateColor(const cv::Scalar& color);

    float x_ = 0;
    float y_ = 0;
    float z_ = 0;
    
    cv::Scalar color_;
    static int id_counter_;
    int id_;
    std::unordered_map<int, int> observers_;
};

#endif