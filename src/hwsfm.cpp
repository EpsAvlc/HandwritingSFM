/*
 * Created on Tue Sep 24 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "hwsfm.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void HWSFM::AddImages(Mat& img)
{
    Frame cur(img);
    frames_.push_back(cur);
}

void HWSFM::SetCameraIntrins(cv::Mat& K)
{
    K_ = K.clone();
}

void HWSFM::StartReconstruction()
{
    if(frames_.size() < 3)
    {
        cerr << "HWFSM error: too few image to reconstruction" << endl;
    }

    initScale();
}

void HWSFM::initScale()
{
    vector<DMatch> good_matches;
    matchFeatures(frames_[0], frames_[1], good_matches);

    vector<Point2f> points_lhs, points_rhs;
    for(int i = 0; i < good_matches.size(); i++)
    {
        points_lhs.push_back(frames_[0].Keypoints()[good_matches[i].queryIdx].pt);
        points_rhs.push_back(frames_[1].Keypoints()[good_matches[i].trainIdx].pt);
    }

    vector<uchar> status;
    Mat essential = findEssentialMat(points_lhs, points_rhs, K_, RANSAC, 0.9, 1, status);

    reduceVector(good_matches, status);

    /* display matches */
    Mat match_mat;
    drawMatches(frames_[0].Img(), frames_[0].Keypoints(), frames_[1].Img(), frames_[1].Keypoints(), good_matches, match_mat);
    resize(match_mat, match_mat, Size(), 0.5, 0.5);
    imshow("matches", match_mat);
    waitKey(0);
    /*******************/

    Mat R, t;
    recoverPose(essential, points_lhs, points_rhs, K_, R, t);   

    /* define a random scale, here we define t's norm is 5 */
    t = t*10;
    /* define first frame as orignal position. */
    frames_[0].SetR(Mat::eye(3, 3, CV_32F));
    frames_[0].SetT(Mat::zeros(3, 1, CV_32F));
    frames_[1].SetR(R);
    frames_[1].SetT(t);
}

void HWSFM::matchFeatures(Frame& lhs, Frame& rhs, vector<DMatch>& good_matches)
{
    good_matches.clear();

    vector<DMatch> matches;
    static FlannBasedMatcher f_matcher;
    f_matcher.match(lhs.Descriptor(), rhs.Descriptor(), matches);

    double min_dist = 10000, max_dist = 0;
    for(int i = 0; i < matches.size(); i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    for(int i = 0; i < matches.size(); i++)
    {
        if(matches[i].distance <= 2*min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }
    
    // /* display matches */
    // Mat match_mat;
    // drawMatches(lhs.Img(), lhs.Keypoints(), rhs.Img(), rhs.Keypoints(), good_matches, match_mat);
    // resize(match_mat, match_mat, Size(), 0.5, 0.5);
    // imshow("matches", match_mat);
    // waitKey(0);

    //TODO: triangulation
}

void HWSFM::triangulation(int l_index, int r_index, const vector<DMatch>& good_matches)
{
    Mat lhs_proj_mat, rhs_proj_mat;
    // lhs_proj_mat = 
}

Point3f HWSFM::pixel2Camera(Point2f& pixel_pt)
{
    if(K_.at<float>(0, 0) < 1e-3)
    {   
        cerr << "[pixel2Camera@HWSFM]: You need init camera intrins first." << endl;
        return Point3f(-1, -1, -1);
    }

    Mat homo_pixel_pt(3, 1, CV_32FC1, Scalar(0));
    homo_pixel_pt.at<float>(0, 0) = pixel_pt.x;
    homo_pixel_pt.at<float>(1, 0) = pixel_pt.y;
    homo_pixel_pt.at<float>(2, 0) = 1;

    Mat camera_pt_mat = K_.inv() * homo_pixel_pt;
    Point3f camera_pt;
    camera_pt.x = camera_pt_mat.at<float>(0, 0) / camera_pt_mat.at<float>(2, 0);
    camera_pt.y = camera_pt_mat.at<float>(1, 0) / camera_pt_mat.at<float>(2, 0);
    camera_pt.z = 1;

    return camera_pt;
}