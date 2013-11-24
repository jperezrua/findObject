#ifndef PATTERN_HPP
#define PATTERN_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

#include <opencv2/opencv.hpp>

/**
 * Store the image data and computed descriptors of target pattern
 */
struct Pattern
{
  cv::Size                  size;
  //cv::Mat                   frame;
  //cv::Mat                   grayImg;

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat                   descriptors;

  std::vector<cv::Point2f>  points2d;
  std::vector<cv::Point3f>  points3d;
};

/**
 * Intermediate pattern tracking info structure
 */
struct PatternTrackingInfo
{
  cv::Mat                   homography;
  std::vector<cv::Point2f>  points2d;
  Transformation            pose3d;
  int                       patternIdx;

  void draw2dContour(cv::Mat& image, cv::Scalar color) const;
  void draw2dPoints(cv::Mat& image, cv::Scalar color) const;
  cv::Rect getRect(cv::Mat& image) const;

  /**
   * Compute pattern pose using PnP algorithm
   */
  void computePose(const Pattern& pattern, const CameraCalibration& calibration);
};

#endif
