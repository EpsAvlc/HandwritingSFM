/*
 * Created on Sat Oct 05 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "viewer.h"
#include "hwsfm.h"
#include "setting.h"

#include <iostream>

#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void Viewer::Run()
{
    pangolin::CreateWindowAndBind("HandwritingSFM: Map Viewer", 1280, 720);
    // To use 3D mousee handler.
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1280,720, setting_.viewer_point_f_, setting_.viewer_point_f_,720,360,0.2,100),
        pangolin::ModelViewLookAt(setting_.viewer_point_x_, setting_.viewer_point_y_, setting_.viewer_point_z_, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.f/480.f).SetHandler(&handler);

        // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    while( !pangolin::ShouldQuit() )
    {
        // Clear the window by current color.
        
        d_cam.Activate(s_cam);

        if(update_)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            drawMappoints();
        }

        pangolin::FinishFrame();
        // usleep(30000);
        Mat cur_match_img = sfm_->GetCurMatch();
        if(!cur_match_img.empty())
        {
            lock_guard<mutex> lock(sfm_->viewer_mutex_);
            imshow("cur_match", cur_match_img);
        }
        waitKey(30);
    }
}

void Viewer::drawMappoints()
{
    lock_guard<mutex> lock(sfm_->viewer_mutex_);
    if(sfm_ == nullptr)
    {
        cerr << "[drawMappoints@Viewer]: Your sfm class is a nullptr." << endl;
        return;
    }
    const vector<MapPoint> mpts = sfm_->GetMappoints(); 
    if(mpts.empty())
        return;

    // cout << mps.size() << endl;
    // set points' radius.
    glPointSize(2);
    glBegin(GL_POINTS);
    // set points' color
    glColor3f(1.0, 0.0, 0.0);

    for(int i = 0; i < mpts.size(); i++)
    {
        Scalar mpt_color = mpts[i].GetColor();
        // Note in opencv color order is BGR.
        glColor3f(float(mpt_color(2) / 255.), float(mpt_color(1) / 255.), float(mpt_color(0) / 255.));
        Point3f world_pos = mpts[i].GetWorldPos();  
        glVertex3f(world_pos.x, world_pos.y, world_pos.z);         
    }
    glEnd();
}