/*
 * Created on Thu Sep 26 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "image.h"

#include <opencv2/xfeatures2d/nonfree.hpp> 
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int Image::id_ = 0;

void Image::extractFeatures(Mat& img)
{
    Ptr<xfeatures2d::SIFT> sift;
    sift->detect(img, keypoints_);
    sift->compute(img, keypoints_, descritor_);

    Mat img_with_features;
    drawKeypoints(img, keypoints_, img_with_features);
    imshow("imgWithFeatures", img_with_features);
    waitKey(0);
}