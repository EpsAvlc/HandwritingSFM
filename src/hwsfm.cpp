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

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace cv;
using namespace pcl;

#define SAVE_POSE


HWSFM::HWSFM(Setting& s) : setting_(s), viewer_(s)
{
    /* Parameters Initialization*/
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
    /*Inital the matches vector*/
    all_matches_.resize(setting_.image_strs_.size());
    for(auto& v: all_matches_)
    {
        v.resize(setting_.image_strs_.size());
    }

    if(frames_.size() < 3)
    {
        cerr << "[StartReconstruction@HWSFM]: too few image to reconstruction. At least 3 images." << endl;
    }

    all_matches_[0][1] = move(initScale());

    for(int i = 2; i < frames_.size(); i++)
    {
        all_matches_[0][i] = move(solvePnPAndTriangulation(frames_[0], frames_[i]));
    }

    for(int i = 1; i < frames_.size()-1; i++)
    {
        for(int j = i+1; j < frames_.size(); j++)
        {
            vector<DMatch> matches;
            matchFeatures(frames_[i], frames_[j], matches);
            rejectWithF(frames_[i], frames_[j], matches);
            triangulation(frames_[i], frames_[j], matches);
            all_matches_[i][j] = move(matches);
        }
    }

    statisticalOutlierRemoval();
    bundleAdjustment();
#ifdef SAVE_POSE
    FileStorage fs("/home/cm/Projects/handwritingSFM/poses/poses.xml", FileStorage::WRITE);
    for(int i = 0; i < frames_.size(); i++)
    {
        string R_name = "R_" + to_string(i);
        fs << R_name << frames_[i].GetR();
        string t_name = "t_" + to_string(i);
        fs << t_name << frames_[i].GetT();
    }
#endif
    statisticalOutlierRemoval();
}

vector<DMatch> HWSFM::initScale()
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

    Mat R, t;
    recoverPose(essential, points_lhs, points_rhs, K_, R, t);   
    R.convertTo(R, CV_32F);
    t.convertTo(t, CV_32F);

    /* define a random scale, here we define t's norm is 0.5 */
    t = t*0.5;
    /* define first frame as orignal position. */
    frames_[0].SetR(Mat::eye(3, 3, CV_32F));
    frames_[0].SetT(Mat::zeros(3, 1, CV_32F));
    frames_[1].SetR(R);
    frames_[1].SetT(t);

    triangulation(frames_[0], frames_[1], good_matches);

    return good_matches;
}

vector<DMatch> HWSFM::solvePnPAndTriangulation(Frame& lhs, Frame& rhs)
{
    if(rhs.IsComputed())
    {
        cerr << "[solvePnPAndTriangulation@HWSFM]: frame " << rhs.Id() << " has been computed." << endl;
        throw(runtime_error("[solvePnPAndTriangulation@HWSFM]"));
    }
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
            pts_3d.push_back(Point3f(mappoints_[mptId].x(), mappoints_[mptId].y(), mappoints_[mptId].z()));
            pts_2d.push_back(rhs.Keypoints()[matches[i].trainIdx].pt);
            mptIds.push_back(mptId);
        }
    }
    if(pts_3d.size() < 8)
    {
        cerr << "[solvePnpAndTriangulation@HWSFM]: too few point pairs betwean frame " << lhs.Id() << " and frame " << rhs.Id() << ", the pairs are " << pts_3d.size() << endl; 
        throw(runtime_error("[solvePnPAndTriangulation@HWSFM]"));
    }
    Mat rvec, t;
    Mat inliers;
    // double confidence = 0.9;
    // solvePnP(pts_3d, pts_2d, K_, noArray(), rvec, t, false, SOLVEPNP_EPNP);
    bool pnp_res = solvePnPRansac(pts_3d, pts_2d, K_, cv::noArray(), rvec, t, false, 1000, 8.f, 0.99, inliers);
    // reduceVector(matches, status);
    // cout << inliers << endl;
    // cout << (float)inliers.rows / pts_3d.size() << endl;
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

    /**************/

    triangulation(lhs, rhs, matches);
    return matches;
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
    // lock_guard<mutex> lock(viewer_mutex_);
    // drawMatches(lhs.Img(), lhs.Keypoints(), rhs.Img(), rhs.Keypoints(), matches, cur_match_img_);
    // resize(cur_match_img_, cur_match_img_, Size(), 0.5, 0.5);
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
    for(int i = 0; i < good_matches.size(); i++)
    {
        int lhs_index = good_matches[i].queryIdx;
        int rhs_index = good_matches[i].trainIdx;
        /** If this point has been triangulated. */
        int mptId = lhs.GetTriangulated(lhs_index);
        if(mptId  != -1)
        {
            rhs.AddTriangulated(rhs_index, mptId);
            mappoints_[mptId].AddObserver(rhs, rhs_index);
            continue;
        }
        mptId = rhs.GetTriangulated(rhs_index);
        if(mptId != -1)
        {
            lhs.AddTriangulated(lhs_index, mptId);
            mappoints_[mptId].AddObserver(lhs, lhs_index);
            continue;
        }
        lhs_cam_pts.push_back(pixel2Camera(lhs.Keypoints()[lhs_index].pt));
        rhs_cam_pts.push_back(pixel2Camera(rhs.Keypoints()[rhs_index].pt));
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
        if(err > 5)
            continue;
        /****************************** */
        mpt.AddObserver(lhs, good_matches[i].queryIdx);
        mpt.AddObserver(rhs, good_matches[i].trainIdx);

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

void HWSFM::bundleAdjustment()
{
    const int kFramesNum = frames_.size();

    cout << "[bundleAdjustment@HWSFM]: Start bundleAdjustment" << endl;
    /*Step 1: Construct a linear solver*/
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    /*Step 2: Add pose vertices*/
    for(int i = 0; i < kFramesNum; i++)
    {
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(i);
        g2o::SE3Quat estimated_pose;
        Mat R_cv = frames_[i].GetR();
        Eigen::Map<Eigen::Matrix3f> R_eigen_f((float*)R_cv.data);
        Eigen::Matrix3d R_eigen_d = R_eigen_f.cast<double>();
        Eigen::Quaterniond q_eigen(R_eigen_d);
        estimated_pose.setRotation(q_eigen);

        Mat t_cv = frames_[i].GetT();
        Eigen::Map<Eigen::Vector3f> t_eigen_f((float*)t_cv.data);
        Eigen::Vector3d t_eigen_d = t_eigen_f.cast<double>();
        estimated_pose.setTranslation(t_eigen_d);

        pose->setEstimate(estimated_pose);
        if(i == 0)
            pose->setFixed(true);

        optimizer.addVertex(pose);
    }

    /*Step 3: Add point vertices*/
    for(auto& mpt: mappoints_)
    {
        g2o::VertexSBAPointXYZ* pt = new g2o::VertexSBAPointXYZ();
        pt->setId(mpt.first + kFramesNum);
        Eigen::Vector3d pt_loc;
        pt_loc.x() = mpt.second.x();
        pt_loc.y() = mpt.second.y();
        pt_loc.z() = mpt.second.z();
        pt->setEstimate(Eigen::Vector3d(pt_loc));
        pt->setMarginalized(true);
        optimizer.addVertex(pt);
    }

    /*Step 4: Add CameraParameters*/
    float focal_length = K_.at<float>(0, 0);
    float cx = K_.at<float>(0, 2);
    float cy = K_.at<float>(1, 2);
    g2o::CameraParameters* cam_param = new g2o::CameraParameters(focal_length, Eigen::Vector2d(cx, cy), 0);
    cam_param->setId(0);
    optimizer.addParameter(cam_param);

    /*Step 5: Add Edges*/
    int edge_index = 1;
    for(auto& mpt: mappoints_)
    {
        const int kMptIndex = mpt.first + kFramesNum;
        unordered_map<int, int> observers = mpt.second.GetObservers();
        for(auto observer:observers)
        {
            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV(); 
            edge->setId(edge_index);
            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(kMptIndex)));
            edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(observer.first)));

            Eigen::Vector2d pt;
            KeyPoint kpt = frames_[observer.first].Keypoints()[observer.second];
            pt.x() = kpt.pt.x;
            pt.y() = kpt.pt.y;
            edge->setMeasurement(pt);
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setParameterId(0, 0);
            edge->setRobustKernel(new g2o::RobustKernelHuber());

            edge_index ++;

            optimizer.addEdge(edge);
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(200);

    lock_guard<mutex> lock(viewer_mutex_);
    for(int i = 0; i < kFramesNum; i++)
    {
        g2o::SE3Quat se3 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i))->estimate();
        frames_[i].SetPose(se3);
    }
    for(auto& mpt:mappoints_)
    {
        const int kMptIndex = mpt.first + kFramesNum;
        Eigen::Vector3d pt_loc = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(kMptIndex))->estimate();
        mpt.second.x() = pt_loc.x();
        mpt.second.y() = pt_loc.y();
        mpt.second.z() = pt_loc.z();
    }
}

void HWSFM::statisticalOutlierRemoval()
{
    const vector<MapPoint> mpts = GetMappoints();
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    for(int i = 0; i < mpts.size(); i++)
    {
        Point3f pt = mpts[i].GetWorldPos();
        cloud->push_back(PointXYZ(pt.x, pt.y, pt.z));
    }

    StatisticalOutlierRemoval<PointXYZ> sor(true);
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.1);
    sor.setInputCloud(cloud);

    PointCloud<PointXYZ> cloud_out;
    sor.filter(cloud_out);
    PointIndices remove_indices;
    sor.getRemovedIndices(remove_indices);
    lock_guard<mutex> lock(viewer_mutex_);
    cout << "[statisticalOutlierRemoval@HWSFM]: There is " << remove_indices.indices.size() << " outliers." << endl;
    cout << "[statisticalOutlierRemoval@HWSFM]: Start remove outliers." << endl;
    for(int i = 0; i < remove_indices.indices.size(); i++)
    {
        int remove_mpt_id = mpts[remove_indices.indices[i]].Id();
        mappoints_.erase(remove_mpt_id);
    }
}