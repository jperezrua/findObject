#ifndef NODE_HPP
#define NODE_HPP

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Quaternion.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <PatternDetector.hpp>

#include <algorithm>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GridCells.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


enum STATE_VAR{
    _DEFAULT,
    _OBJECT_NOT_FOUND,
    _OBJECT_FOUND,
    _WAITING_POSE,
    _DIFF_POSE_NOT_REACHED,
    _DIFF_POSE_REACHED,
    _WAITING_TARGET,
    _TARGET_NOT_REACHABLE,
    _TARGET_REACHED,
    _ROBUST_OBJECT_FOUND,
    _ROBUST_OBJECT_NOT_FOUND
};

class ObjectFinder{
public:
    ObjectFinder();

    void applyAction();
private:

    STATE_VAR _CURRENT_STATE;

    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher gopose_pub_;
    ros::Publisher diff_pub_;
    ros::Publisher vel_pub_;
    ros::Subscriber cam_info_;
    ros::Subscriber mapSub;
    //ros::Subscriber occSub;
    image_transport::Subscriber ima_sub_;
    image_transport::Subscriber dep_sub_;
    image_transport::Publisher ima_pub_;
    image_transport::ImageTransport *it;

    cv::Mat rgb_im;
    cv::Mat dep_im;
    cv::Mat templ;
    cv::Mat mapf;
    std::string depth_node_name;
    std::string rgb_node_name;
    std::string caminfo_node_name;
    std::string template_name;
    std::string kinect_frame_name;
    std::string fixed_frame;

    std::vector<cv::Point> pathGraph;
    boost::array<double, 9ul> kam;
    tf::TransformListener listener;
    tf::TransformListener listener2;
    tf::StampedTransform bl2CamTf;
    tf::StampedTransform bl2CamTf2;
    bool im_ready;
    bool dep_ready;
    bool kam_ready;
    double findObjectYaw(const cv::Mat &depth, const cv::Mat &mask,
                         double alpha_x, double u0, const cv::Rect &rec);
    void readImage(const sensor_msgs::ImageConstPtr& kinectImage);
    void readDepth(const sensor_msgs::ImageConstPtr& kinectImage);
    void readKam(const sensor_msgs::CameraInfoConstPtr& camInfo);
    void detectObject(const cv::Mat& I, std::vector<cv::Point> &objectCoor);
    void goalDone(const actionlib::SimpleClientGoalState &state);
    void mapper(const nav_msgs::OccupancyGridPtr &map);
    void mapperobs(const nav_msgs::GridCellsPtr& cells);
    std::vector<cv::Point> getMostSimilObj(std::vector<std::vector<Point> > squares, const cv::Mat I);
    Rect getBB(std::vector<cv::Point> obj);
    geometry_msgs::Quaternion getRotMat(CameraCalibration cc, cv::Size sz, std::vector<Point2f> points2d);
    void pather();
    size_t currPathIdx;
    MoveBaseClient *ac;
    bool moving;
    bool targetReached;
    bool mapfready;
    bool firsttime;
    double yawAnglePose;
    int findops;
    cv::Point init_point;
};

#endif
