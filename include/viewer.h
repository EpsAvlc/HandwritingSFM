/*
 * Created on Sat Oct 05 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef VIEWER_H__
#define VIEWER_H__

class HWSFM;

class Viewer
{
public:
    Viewer() {};
    void Run();
    void SetSFM(HWSFM* sfm) {
        sfm_ = sfm;
    };
private:
    void drawMappoints();
    HWSFM* sfm_ = nullptr;
};
    

#endif