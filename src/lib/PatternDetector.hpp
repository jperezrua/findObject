#ifndef PATTERNDETECTOR_HPP
#define PATTERNDETECTOR_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "Pattern.hpp"

#include <opencv2/opencv.hpp>

class PatternDetector
{
public:
    /**
     * Initialize a pattern detector with specified feature detector, descriptor extraction and matching algorithm
     */
    PatternDetector
        (
        cv::Ptr<cv::FeatureDetector>     detector  =  new cv::ORB(600),
        cv::Ptr<cv::DescriptorExtractor> extractor =  new cv::ORB(600),
        bool enableRatioTest                       = false
        );

    /**
    * 
    */
    void train(const std::vector<Pattern>& pattern);

    /**
    * Initialize Pattern structure from the input image.
    * This function finds the feature points and extract descriptors for them.
    */
    void buildPatternsFromImages(const std::vector<cv::Mat>& images, std::vector<Pattern>& patterns) const;
    void buildPatternsFromYAML(const std::vector<std::string>& files, std::vector<Pattern>& patterns) const;

    /**
    * Tries to find a @pattern object on given @image. 
    * The function returns true if succeeded and store the result (pattern 2d location, homography) in @info.
    */
    bool findPattern(const cv::Mat& image, PatternTrackingInfo& info);

    bool enableRatioTest;
    bool enableHomographyRefinement;
    float homographyReprojectionThreshold;

protected:

    bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

    void findPatternMatch(const cv::Mat queryDescriptors, int patternNumber);
    void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches, int patternIdx);

    /**
    * Get the gray image from the input image.
    * Function performs necessary color conversion if necessary
    * Supported input images types - 1 channel (no conversion is done), 3 channels (assuming BGR) and 4 channels (assuming BGRA).
    */
    static void getGray(const cv::Mat& image, cv::Mat& gray);

    /**
    * 
    */
    static bool refineMatchesWithHomography(
        const std::vector<cv::KeyPoint>& queryKeypoints, 
        const std::vector<cv::KeyPoint>& trainKeypoints, 
        float reprojectionThreshold,
        std::vector<cv::DMatch>& matches, 
        cv::Mat& homography);

private:
    std::vector<cv::KeyPoint> m_queryKeypoints;
    cv::Mat                   m_queryDescriptors;

    std::vector<std::vector<cv::DMatch> > m_matches;
    std::vector<bool> m_matches_homographyFound;
    std::vector<cv::Mat> m_matches_homography;

    cv::Mat                   m_grayImg;
    cv::Mat                   m_warpedImg;
    cv::Mat                   m_roughHomography;
    cv::Mat                   m_refinedHomography;

    std::vector<Pattern>             m_patterns;
    Pattern                          m_pattern;
    cv::Ptr<cv::FeatureDetector>     m_detector;
    cv::Ptr<cv::DescriptorExtractor> m_extractor;
    std::vector<cv::Ptr<cv::DescriptorMatcher> > m_matchers;

    class PatternMatch : public cv::ParallelLoopBody {
        cv::Mat queryDescriptors;
        PatternDetector& parent;

    public:
        PatternMatch(cv::Mat& queryDescriptors, PatternDetector& parent) : queryDescriptors(queryDescriptors), parent(parent) {}

        void operator() (const cv::Range& range) const {
            for (int i = range.start; i != range.end; i++) {
                parent.findPatternMatch(queryDescriptors, i);
            }
        }
    };
};

#endif
