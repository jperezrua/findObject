////////////////////////////////////////////////////////////////////
// File includes:
#include "Pattern.hpp"

void PatternTrackingInfo::computePose(const Pattern& pattern, const CameraCalibration& calibration)
{
  cv::Mat Rvec;
  cv::Mat_<float> Tvec;
  cv::Mat raux,taux;

  cv::solvePnP(pattern.points3d, points2d, calibration.getIntrinsic(), calibration.getDistorsion(),raux,taux);
  raux.convertTo(Rvec,CV_32F);
  taux.convertTo(Tvec ,CV_32F);

  cv::Mat_<float> rotMat(3,3); 
  cv::Rodrigues(Rvec, rotMat);

  // Copy to transformation matrix
  for (int col=0; col<3; col++)
  {
    for (int row=0; row<3; row++)
    {        
     pose3d.r()(row,col) = rotMat(row,col); // Copy rotation component
    }
    pose3d.t()[col] = Tvec(col); // Copy translation component
  }

  // Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
  pose3d = pose3d.getInverted();
}

void PatternTrackingInfo::draw2dContour(cv::Mat& image, cv::Scalar color) const
{
  for (size_t i = 0; i < points2d.size(); i++)
  {
    cv::line(image, points2d[i], points2d[ (i+1) % points2d.size() ], color, 2, CV_AA);
  }
}

void PatternTrackingInfo::draw2dPoints(cv::Mat& image, cv::Scalar color) const
{
  for (size_t i = 0; i < points2d.size(); i++)
  {
    cv::circle(image, points2d[i], 3, color, 2);
  }
}

cv::Rect PatternTrackingInfo::getRect(cv::Mat& image)  const{
    float minx=std::numeric_limits<float>::max();
    float maxx=std::numeric_limits<float>::min();
    float miny=std::numeric_limits<float>::max();
    float maxy=std::numeric_limits<float>::min();

    for (size_t i = 0; i < points2d.size(); i++){
        if(points2d[i].x<minx){
            minx=points2d[i].x;
        }
        if(points2d[i].x>maxx){
            maxx=points2d[i].x;
        }
        if(points2d[i].y<miny){
            miny=points2d[i].y;
        }
        if(points2d[i].y>maxy){
            maxy=points2d[i].y;
        }
    }
    if (maxx>image.cols) maxx=image.cols-1;
    if (maxy>image.rows) maxy=image.rows-1;
    if (minx<1) minx=1;
    if (miny<1) miny=1;
    cv::Rect out(Point2f(minx,miny),Point2f(maxx,maxy));
    return out;
}





