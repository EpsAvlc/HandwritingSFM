/*
 * Created on Sat Oct 05 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "mappoint.h"

using namespace cv;
using namespace std;

bool MapPoint::AddObserver(Frame& frame, int feature_index)
{
    if(observers_.count(frame.Id()) == 0)
    {
        observers_[frame.Id()] = feature_index;
        Scalar color = frame.Img().at<Vec3b>(frame.Keypoints()[feature_index].pt);
        updateColor(color);        
        return true;
    }
    else
    {
        return false;
    }
    
}

int MapPoint::QueryObserver(int frame_index)
{
    if(observers_.count(frame_index) == 0)
    {
        return -1;
    }
    else{
        return observers_[frame_index];
    }
}

void MapPoint::updateColor(const cv::Scalar& color)
{
   long colors[3] = {};
   /* Only One observer */
   if(observers_.size() == 1) 
   {
       color_ = color;
   }
   else
   {
        int n = observers_.size() - 1;
        for(int i = 0; i < 3; i++)
        {
            colors[i] = (long(color_(i)) * n + color(i)) / (n+1);
            color_(i) = colors[i];
        }
   }
   
}

int MapPoint::id_counter_ = 0;