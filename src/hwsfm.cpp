#include "hwsfm.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

using namespace std;
using namespace cv;

void HWSFM::AddImages(Mat& img)
{
    // currently we only consider image with same size.
    if(img.cols != 4368)
        return;
    
    Frame cur(img);
    frames_.push_back(cur);
}

void HWSFM::StartReconstruction()
{
    if(frames_.size() < 3)
    {
        cerr << "HWFSM error: too few image to reconstruction" << endl;
    }

    initScale();

}

/**
 * @brief Use first two images to init scale.
 * 
 */
void HWSFM::initScale()
{
    vector<DMatch> good_matches;
    matchFeatures(frames_[0], frames_[1], good_matches);

    Mat img_match;
    drawMatches(frames_[0].Img(), frames_[0].Keypoints(), frames_[1].Img(), frames_[1].Keypoints(), good_matches, img_match);
    resize(img_match, img_match, Size(), 0.2, 0.2);
    imshow("img_match", img_match);
    waitKey(0);
}

void HWSFM::matchFeatures(Frame& lhs, Frame& rhs, vector<DMatch>& good_matches)
{
    good_matches.clear();

    static FlannBasedMatcher f_matcher;
    vector<DMatch> matches;
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
        if(matches[i].distance <= 2*min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

}