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

class MapPoint
{
public:
    MapPoint(){};
    MapPoint(double x, double y, double z) : x_(x), y_(y), z_(z){};
    double& x(){return x_;};
    double& y(){return y_;};
    double& z(){return z_;};
private:
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
    // std::vector<MapPoint>
};

#endif