////////////////////////////////////////////////////////////////////
// File includes:
#include "Tracker.hpp"

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <cmath>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <opencv2/highgui/highgui.hpp>

float range1[] = { 0, 256 } ;
float range2[] = { 0, 256 } ;
float range3[] = { 0, 256 } ;
const float* histRange[] = { range1, range2, range3 };
int channels[] = {0,1,2};

Tracker::Tracker(int histSize){
    hsize=histSize;
    started=false;
}

void Tracker::SetTracker(cv::Mat train_image, cv::Rect obj){
    cv::Mat new_space;
    cv::cvtColor(train_image, new_space, CV_BGR2Lab);
    cv::Mat roi(new_space, obj);
    trackWindow=obj;
    int arraySz[3]={hsize, hsize, hsize};
    cv::calcHist(&roi, 1, channels, cv::Mat(), targetModel, 3, arraySz, histRange, true, false);
    //cv::normalize(targetModel, targetModel, 0, 255, CV_MINMAX);
    started=true;
}

cv::RotatedRect Tracker::track(cv::Mat input_image){
    cv::Mat new_space;
    cv::cvtColor(input_image, new_space, CV_BGR2Lab);
    cv::calcBackProject(&new_space, 1, channels, targetModel, backproj, histRange);
    return CamShift(backproj, trackWindow,
        cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
}
