#include <opencv2/xfeatures2d/nonfree.hpp>
#include "hwsfm.h"

using namespace std;
using namespace cv;

void HWSFM::AddImages(Mat& img)
{
    // currently we only consider image with same size.
    if(img.cols != 4368)
        return;
    
    Image cur(img);
    imgs_.push_back(cur);
}

void HWSFM::StartReconstruction()
{
    // extractFeatures();
}

void HWSFM::extractFeatures()
{
    Ptr<xfeatures2d::SIFT> sift;
    for(int i = 0; i < imgs_.size(); i++)
    {

    }
}