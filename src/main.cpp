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

    HWSFM sfm(s);

    sfm.StartReconstruction();
    while(1);
}
