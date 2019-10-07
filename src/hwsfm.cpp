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

HWSFM::HWSFM()
{
    K_ = Mat::zeros(3, 3, CV_32F);
    viewer_.SetSFM(this);
    viewer_thread_ = thread(&Viewer::Run, &viewer_);

}

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
    // Mat match_mat;
    // drawMatches(frames_[0].Img(), frames_[0].Keypoints(), frames_[1].Img(), frames_[1].Keypoints(), good_matches, match_mat, Scalar::all(-1), Scalar::all(-1), vector<char>(),  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // resize(match_mat, match_mat, Size(), 0.5, 0.5);
    // imshow("matches", match_mat);
    // waitKey(0);
    /*******************/

    Mat R, t;
    recoverPose(essential, points_lhs, points_rhs, K_, R, t);   
    R.convertTo(R, CV_32F);
    t.convertTo(t, CV_32F);

    /* define a random scale, here we define t's norm is 5 */
    // t = t*5;
    /* define first frame as orignal position. */
    frames_[0].SetR(Mat::eye(3, 3, CV_32F));
    frames_[0].SetT(Mat::zeros(3, 1, CV_32F));
    frames_[1].SetR(R);
    frames_[1].SetT(t);

    triangulation(0, 1, good_matches);
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
        if(matches[i].distance <= 4*min_dist)
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
}

void HWSFM::triangulation(int l_index, int r_index, const vector<DMatch>& good_matches)
{
    const Mat& lhs_R = frames_[l_index].GetR();
    const Mat& lhs_t = frames_[l_index].GetT();
    Mat lhs_proj_mat = (Mat_<float>(3, 4) <<
        lhs_R.at<float>(0, 0), lhs_R.at<float>(0, 1), lhs_R.at<float>(0, 2), lhs_t.at<float>(0, 0),
        lhs_R.at<float>(1, 0), lhs_R.at<float>(1, 1), lhs_R.at<float>(1, 2), lhs_t.at<float>(1, 0),
        lhs_R.at<float>(2, 0), lhs_R.at<float>(2, 1), lhs_R.at<float>(2, 2), lhs_t.at<float>(2, 0));

    const Mat& rhs_R = frames_[r_index].GetR();
    const Mat& rhs_t = frames_[r_index].GetT();
    Mat rhs_proj_mat = (Mat_<float>(3, 4) <<
        rhs_R.at<float>(0, 0), rhs_R.at<float>(0, 1), rhs_R.at<float>(0, 2), rhs_t.at<float>(0, 0),
        rhs_R.at<float>(1, 0), rhs_R.at<float>(1, 1), rhs_R.at<float>(1, 2), rhs_t.at<float>(1, 0),
        rhs_R.at<float>(2, 0), rhs_R.at<float>(2, 1), rhs_R.at<float>(2, 2), rhs_t.at<float>(2, 0));

    vector<Point2f> lhs_cam_pts, rhs_cam_pts;
    for(int i = 0; i < good_matches.size(); i++)
    {
        lhs_cam_pts.push_back(pixel2Camera(frames_[l_index].Keypoints()[good_matches[i].queryIdx].pt));
        rhs_cam_pts.push_back(pixel2Camera(frames_[r_index].Keypoints()[good_matches[i].trainIdx].pt));
    }

    Mat pts4d;
    triangulatePoints(lhs_proj_mat, rhs_proj_mat, lhs_cam_pts, rhs_cam_pts, pts4d);
    for(int i = 0; i < pts4d.cols; i++)
    {
        Mat pt = pts4d.col(i);
        pt /= pt.at<float>(3, 0);
        MapPoint mpt(pt.at<float>(0, 0), pt.at<float>(1, 0), pt.at<float>(2, 0));

        Scalar mpt_color = frames_[l_index].Img().at<Vec3b>(frames_[l_index].Keypoints()[good_matches[i].queryIdx].pt);
        mpt.SetColor(mpt_color);
        mpt.AddObserver(l_index, good_matches[i].queryIdx);
        mpt.AddObserver(r_index, good_matches[i].trainIdx);

        frames_[l_index].AddTriangulated(good_matches[i].queryIdx, mpt.Id());
        frames_[r_index].AddTriangulated(good_matches[i].trainIdx, mpt.Id());

        mappoints_.push_back(mpt); 

        // cout << mpt.x() << " " << mpt.y() << " " << mpt.z() << endl;
        /********** validation **********/
        // Mat pt3d(3, 1, CV_32FC1);
        // pt3d.at<float>(0, 0) = pt.at<float>(0, 0);
        // pt3d.at<float>(1, 0) = pt.at<float>(1, 0);
        // pt3d.at<float>(2, 0) = pt.at<float>(2, 0);

        // Mat reprojected = (frames_[l_index].GetR() * pt3d + frames_[l_index].GetT());
        // reprojected /= reprojected.at<float>(2, 0); 

        // cout << "left point original: " << frames_[l_index].Keypoints()[good_matches[i].queryIdx].pt << endl;
        // // cout << "left point orignal: " << lhs_cam_pts[i] << endl;
        // cout << "left point reporjected: " << K_* reprojected << endl; 
        /****************************** */

    }
}

Point2f HWSFM::pixel2Camera(const Point2f& pixel_pt)
{
    if(K_.at<float>(0, 0) < 1e-3)
    {   
        cerr << "[pixel2Camera@HWSFM]: You need init camera intrins first." << endl;
        return Point2f(-1, -1);
    }

    return Point2d
        (
            ( pixel_pt.x - K_.at<float> ( 0,2 ) ) / K_.at<float> ( 0,0 ),
            ( pixel_pt.y - K_.at<float> ( 1,2 ) ) / K_.at<float> ( 1,1 )
        );
}