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

class Setting;
class HWSFM;

class Viewer
{
public:
    Viewer(Setting& s) : setting_(s){};
    void Run();
    void SetSFM(HWSFM* sfm) {
        sfm_ = sfm;
    };
    void SetUpdate() {update_ = true;};
private:
    void drawMappoints();
    void drawCameras();
    HWSFM* sfm_ = nullptr;
    Setting& setting_;
    bool update_ = false;
};
    

#endif