#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "setting.h"
#include "mappoint.h"
#include "hwsfm.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    Setting s("./config/sceauxcastle.yaml");
    vector<Mat> imgs;
    vector<string> img_paths = s.imagePaths();
    if(img_paths.size() == 0)
    {
        cerr << "Faied to get image paths. Please check your yaml file" << endl; 
        return 1;
    }

    HWSFM sfm;
    sfm.SetCameraIntrins(s.K());
    vector<KeyPoint> keypoints_;
    for(int i = 0; i < img_paths.size(); i ++)
    {
        Mat tmp = imread(img_paths[i]);
        sfm.AddImages(tmp);
    }

    sfm.StartReconstruction();
}
