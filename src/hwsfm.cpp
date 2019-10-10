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

HWSFM::HWSFM(Setting& s) : setting_(s), viewer_(s)
{
    K_ = setting_.K_.clone();
    viewer_.SetSFM(this);
    viewer_thread_ = thread(&Viewer::Run, &viewer_);
}

void HWSFM::addImages(Mat& img)
{
    Frame cur(img);
    cout << "[addImages@HWSFM]: Finishing adding image " << cur.Id() << endl;
    frames_.push_back(cur);
}

void HWSFM::StartReconstruction()
{
    for(int i = 0; i < setting_.image_strs_.size(); i++)
    {
        Mat img = imread(setting_.image_strs_[i]);
        addImages(img);
    }
    if(frames_.size() < 3)
    {
        cerr << "[StartReconstruction@HWSFM]: too few image to reconstruction" << endl;
    }

    initScale();
    viewer_.SetUpdate();

    for(int i = 2; i < frames_.size(); i++)
    {
        solvePnPAndTriangulation(frames_[0], frames_[i]);
        viewer_.SetUpdate();
    }

    for(int i = 1; i < frames_.size()-1; i++)
    {
        for(int j = i+1; j < frames_.size(); j++)
        {
            vector<DMatch> matches;
            matchFeatures(frames_[i], frames_[j], matches);
            rejectWithF(frames_[i], frames_[j], matches);
            triangulation(frames_[i], frames_[j], matches);
            viewer_.SetUpdate();
        }
    }
}

void HWSFM::initScale()
{
    vector<DMatch> good_matches;
    matchFeatures(frames_[0], frames_[1], good_matches);
    rejectWithF(frames_[0], frames_[1], good_matches);

    vector<Point2f> points_lhs, points_rhs;
    for(int i = 0; i < good_matches.size(); i++)
    {
        points_lhs.push_back(frames_[0].Keypoints()[good_matches[i].queryIdx].pt);
        points_rhs.push_back(frames_[1].Keypoints()[good_matches[i].trainIdx].pt);
    }

    vector<uchar> status;
    Mat essential = findEssentialMat(points_lhs, points_rhs, K_, RANSAC, 0.9, 1, status);

    // reduceVector(good_matches, status);

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

    triangulation(frames_[0], frames_[1], good_matches);
}

void HWSFM::solvePnPAndTriangulation(Frame& lhs, Frame& rhs)
{
    vector<DMatch> matches;
    matchFeatures(lhs, rhs, matches);
    rejectWithF(lhs, rhs, matches);

    /* display matches */
    // Mat match_mat;
    // drawMatches(lhs.Img(), lhs.Keypoints(), rhs.Img(), rhs.Keypoints(), matches, match_mat, Scalar::all(-1), Scalar::all(-1), vector<char>(),  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // resize(match_mat, match_mat, Size(), 0.5, 0.5);
    // imshow("matches", match_mat);
    // waitKey(0);
    /*******************/

    /* start solve pnp */
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    vector<int> mptIds;
    for(int i = 0; i < matches.size(); i++)
    {
        int mptId = lhs.GetTriangulated(matches[i].queryIdx);
        if( mptId != -1)
        {
            // cout << matches[i].queryIdx << endl;
            // cout << mappoints_[mptId].QueryObserver(lhs.Id()) << endl;
            /* validation */
            // Mat pt3d(3, 1, CV_32FC1);
            // pt3d.at<float>(0, 0) = mappoints_[mptId].x();
            // pt3d.at<float>(1, 0) = mappoints_[mptId].y();
            // pt3d.at<float>(2, 0) =  mappoints_[mptId].z();

            // Mat reprojected = K_*(lhs.GetR() * pt3d + lhs.GetT());
            // reprojected /= reprojected.at<float>(2, 0); 
            // cout << "original : " << lhs.Keypoints()[matches[i].queryIdx].pt << endl;
            // cout << "reprojected : " << reprojected << endl;
            /**************/
            pts_3d.push_back(Point3f(mappoints_[mptId].x(), mappoints_[mptId].y(), mappoints_[mptId].z()));
            pts_2d.push_back(rhs.Keypoints()[matches[i].trainIdx].pt);
            mptIds.push_back(mptId);
        }
    }
    if(pts_3d.size() < 8)
    {
        cerr << "[solvePnpAndTriangulation@HWSFM]: too few point pairs betwean frame " << lhs.Id() << " and frame " << rhs.Id() << ", the pairs are " << pts_3d.size() << endl; 
        return;
    }
    Mat rvec, t;
    Mat inliers;;
    // solvePnP(pts_3d, pts_2d, K_, noArray(), rvec, t, false, SOLVEPNP_EPNP);
    solvePnPRansac(pts_3d, pts_2d, K_, cv::noArray(), rvec, t, false, 1000, 8.f, 0.9, inliers);
    // reduceVector(matches, status);
    // cout << inliers << endl;
    Mat R;
    Rodrigues(rvec, R);
    R.convertTo(R, CV_32F);
    t.convertTo(t, CV_32F);
    rhs.SetR(R);
    rhs.SetT(t);

    /* validation */
    // for(int i = 0; i < pts_3d.size(); i++)
    // {
    //     Mat pt3d(3, 1, CV_32FC1);
    //     pt3d.at<float>(0, 0) = pts_3d[i].x;
    //     pt3d.at<float>(1, 0) = pts_3d[i].y;
    //     pt3d.at<float>(2, 0) =  pts_3d[i].z;

    //     Mat reprojected_lhs = K_*(lhs.GetR() * pt3d + lhs.GetT());
    //     Point2f pt_lhs = lhs.Keypoints()[mappoints_[mptIds[i]].QueryObserver(lhs.Id())].pt;
    //     reprojected_lhs /= reprojected_lhs.at<float>(2, 0); 

    //     Mat reprojected_rhs = K_*(rhs.GetR() * pt3d + rhs.GetT());
    //     reprojected_rhs /= reprojected_rhs.at<float>(2, 0); 
    //     cout << "original_lhs : " << pts_2d[i] << endl;
    //     cout << "reprojected_lhs : " << reprojected_lhs << endl;
    //     cout << "original_rhs : " << pts_2d[i] << endl;
    //     cout << "reprojected_rhs : " << reprojected_rhs << endl;
    // }

    // cout << R << endl;
    // cout << t << endl;

    /**************/

    triangulation(lhs, rhs, matches);
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
        if(matches[i].distance <= 8*min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }
    

}

void HWSFM::rejectWithF(Frame& lhs, Frame& rhs, vector<cv::DMatch>& matches)
{
    vector<Point2f> pts_lhs, pts_rhs;
    for(int i = 0; i < matches.size(); i++)
    {
        pts_lhs.push_back(lhs.Keypoints()[matches[i].queryIdx].pt);
        pts_rhs.push_back(rhs.Keypoints()[matches[i].trainIdx].pt);
    }

    vector<uchar> status;
    Mat F = findFundamentalMat(pts_lhs, pts_rhs, RANSAC, 3.0, 0.999, status);
    reduceVector(matches, status);
    // /* display matches */
    lock_guard<mutex> lock(viewer_mutex_);
    drawMatches(lhs.Img(), lhs.Keypoints(), rhs.Img(), rhs.Keypoints(), matches, cur_match_img_);
    resize(cur_match_img_, cur_match_img_, Size(), 0.5, 0.5);
}

void HWSFM::triangulation(Frame& lhs, Frame& rhs, const vector<DMatch>& good_matches)
{
    lock_guard<mutex> lock(viewer_mutex_);
    const Mat& lhs_R = lhs.GetR();
    const Mat& lhs_t = lhs.GetT();
    Mat lhs_proj_mat = (Mat_<float>(3, 4) <<
        lhs_R.at<float>(0, 0), lhs_R.at<float>(0, 1), lhs_R.at<float>(0, 2), lhs_t.at<float>(0, 0),
        lhs_R.at<float>(1, 0), lhs_R.at<float>(1, 1), lhs_R.at<float>(1, 2), lhs_t.at<float>(1, 0),
        lhs_R.at<float>(2, 0), lhs_R.at<float>(2, 1), lhs_R.at<float>(2, 2), lhs_t.at<float>(2, 0));

    const Mat& rhs_R = rhs.GetR();
    const Mat& rhs_t = rhs.GetT();
    Mat rhs_proj_mat = (Mat_<float>(3, 4) <<
        rhs_R.at<float>(0, 0), rhs_R.at<float>(0, 1), rhs_R.at<float>(0, 2), rhs_t.at<float>(0, 0),
        rhs_R.at<float>(1, 0), rhs_R.at<float>(1, 1), rhs_R.at<float>(1, 2), rhs_t.at<float>(1, 0),
        rhs_R.at<float>(2, 0), rhs_R.at<float>(2, 1), rhs_R.at<float>(2, 2), rhs_t.at<float>(2, 0));

    vector<Point2f> lhs_cam_pts, rhs_cam_pts;
    // vector<Point2f> origin_pts;
    for(int i = 0; i < good_matches.size(); i++)
    {
        int lhs_index = good_matches[i].queryIdx;
        int rhs_index = good_matches[i].trainIdx;
        // /** If this point has been triangulated. */
        if(int mptId = lhs.GetTriangulated(lhs_index) != -1)
        {
            rhs.AddTriangulated(rhs_index, mptId);
            continue;
        }
        else if(int mptId = rhs.GetTriangulated(rhs_index) != -1)
        {
            lhs.AddTriangulated(lhs_index, mptId);
            continue;
        }
        lhs_cam_pts.push_back(pixel2Camera(lhs.Keypoints()[lhs_index].pt));
        rhs_cam_pts.push_back(pixel2Camera(rhs.Keypoints()[rhs_index].pt));
        // origin_pts.push_back(lhs.Keypoints()[lhs_index].pt);
    }

    Mat pts4d;
    triangulatePoints(lhs_proj_mat, rhs_proj_mat, lhs_cam_pts, rhs_cam_pts, pts4d);
    int tri_count = 0;
    for(int i = 0; i < pts4d.cols; i++)
    {
        Mat pt = pts4d.col(i);
        pt /= pt.at<float>(3, 0);
        MapPoint mpt(pt.at<float>(0, 0), pt.at<float>(1, 0), pt.at<float>(2, 0));

        /********** Reject points with huge projection error **********/
        Mat pt3d(3, 1, CV_32FC1);
        pt3d.at<float>(0, 0) = pt.at<float>(0, 0);
        pt3d.at<float>(1, 0) = pt.at<float>(1, 0);
        pt3d.at<float>(2, 0) = pt.at<float>(2, 0);

        Mat reprojected = K_ *(lhs.GetR() * pt3d + lhs.GetT());
        reprojected /= reprojected.at<float>(2, 0); 

        Point2f pt2d = camera2Pixel(lhs_cam_pts[i]);
        // cout << reprojected << endl;
        // cout << pt2d << endl;

        float err = fabs(reprojected.at<float>(0, 0) - pt2d.x) + fabs(reprojected.at<float>(1, 0) - pt2d.y);
        if(err > 8)
            continue;
        /****************************** */
        // TODO: add update color;
        Scalar mpt_lhs_color =lhs.Img().at<Vec3b>(lhs.Keypoints()[good_matches[i].queryIdx].pt);
        mpt.SetColor(mpt_lhs_color);
        // Scalar mpt_rhs_color =rhs.Img().at<Vec3b>(lhs.Keypoints()[good_matches[i].queryIdx].pt);
        // mpt.SetColor(mpt_color);
        mpt.AddObserver(lhs.Id(), good_matches[i].queryIdx);
        mpt.AddObserver(rhs.Id(), good_matches[i].trainIdx);
        // mpt.UpdateColor(mpt_lhs_color);
        // mpt.UpdateColor(mpt_rhs_color);

        lhs.AddTriangulated(good_matches[i].queryIdx, mpt.Id());
        rhs.AddTriangulated(good_matches[i].trainIdx, mpt.Id());

        mappoints_[mpt.Id()] = mpt; 
        tri_count ++;
    }
    cout << "[triangulation@HWSFM]: triangulate " << tri_count << " points between frame " << lhs.Id() << " and " << rhs.Id() << endl;
}

Point2f HWSFM::pixel2Camera(const Point2f& pixel_pt)
{
    if(K_.at<float>(0, 0) < 1e-3)
    {   
        cerr << "[pixel2Camera@HWSFM]: You need init camera intrins first." << endl;
        return Point2f(-1, -1);
    }

    return Point2f
        (
            ( pixel_pt.x - K_.at<float> ( 0,2 ) ) / K_.at<float> ( 0,0 ),
            ( pixel_pt.y - K_.at<float> ( 1,2 ) ) / K_.at<float> ( 1,1 )
        );
}

Point2f HWSFM::camera2Pixel(const cv::Point2f& cam_pt)
{
    if(K_.at<float>(0, 0) < 1e-3)
    {   
        cerr << "[camera2Pixel@HWSFM]: You need init camera intrins first." << endl;
        return Point2f(-1, -1);
    }
    return Point2f
    (
        cam_pt.x * K_.at<float> ( 0,0 ) + K_.at<float>(0, 2),
        cam_pt.y * K_.at<float> ( 1,1 ) + K_.at<float>(1, 2)
    );
}

const vector<MapPoint> HWSFM::GetMappoints()
{
    vector<MapPoint> res;
    for(auto it = mappoints_.begin(); it != mappoints_.end(); it++)
    {
        res.push_back(it->second);
    }
    return res;
}