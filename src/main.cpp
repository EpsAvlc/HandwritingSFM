#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "setting.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    Setting s("./config/qinghuamen.yaml");
    vector<Mat> imgs;
    vector<string> img_paths = s.imagePaths();
    if(img_paths.size() == 0)
    {
        cerr << "Faied to get image paths. Please check your yaml file" << endl; 
        return 1;
    }
    for(int i = 0; i < img_paths.size(); i ++)
    {
        Mat tmp = imread(img_paths[i]);
        imgs.push_back(tmp);
    }
    if(imgs[0].empty())
    {
        cerr << "Failed to read image. please check your yaml file." << endl;
        return 1;
    }
    imshow("img", imgs[0]);
    waitKey(0);
}