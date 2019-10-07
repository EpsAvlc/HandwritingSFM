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

#include <iostream>

#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

void Viewer::Run()
{
    pangolin::CreateWindowAndBind("HandwritingSFM: Map Viewer", 1280, 720);
    // To use 3D mousee handler.
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1280,720,420,420,720,360,0.2,100),
        pangolin::ModelViewLookAt(0,2,-70, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.f/480.f).SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        // Clear the window by current color.
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        // glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
        // drawMappoints();

    // glPointSize(5);
    // glBegin(GL_POINTS);
    // // set points' color
    // glColor3f(1.0, 0.0, 0.0);
    // glVertex3d(0, 0, 0);
    // glEnd();

        drawMappoints();

        pangolin::FinishFrame();
        usleep(30000);
    }
}

void Viewer::drawMappoints()
{
    if(sfm_ == nullptr)
    {
        cerr << "[drawMappoints@Viewer]: Your sfm class is a nullptr." << endl;
        return;
    }
    const vector<MapPoint> mps = sfm_->GetMappoints(); 
    if(mps.empty())
        return;

    // set points' radius.
    glPointSize(5);
    glBegin(GL_POINTS);
    // set points' color
    glColor3f(1.0, 0.0, 0.0);

    for(int i = 0; i < mps.size(); i++)
    {
        Point3f world_pos = mps[i].GetWorldPos();  
        glVertex3f(world_pos.x, world_pos.y, world_pos.z);         
    }
    glEnd();
}