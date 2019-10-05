/*
 * Created on Sat Oct 05 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "mappoint.h"

bool MapPoint::AddObserver(int frame_index, int feature_index)
{
    if(observers_.count(frame_index) == 0)
    {
        observers_[frame_index] = feature_index;
        return true;
    }
    else
    {
        return false;
    }
    
}

int MapPoint::id_counter_ = 0;