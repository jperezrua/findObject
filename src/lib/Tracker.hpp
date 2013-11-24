#ifndef TRACKER_HPP
#define TRACKER_HPP


#include <opencv2/opencv.hpp>

class Tracker{
public:
    Tracker(int histSize=16);

    void SetTracker(cv::Mat train_image, cv::Rect obj);
    cv::RotatedRect track(cv::Mat input_image);

    bool started;
private:
    cv::Rect trackWindow;
    cv::RotatedRect trackBox;
    int hsize;
    cv::Mat backproj;
    cv::Mat targetModel;
};

#endif
