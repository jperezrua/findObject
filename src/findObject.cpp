/* * * * * * * * * * * * * * * * * * * *
 * ==========  FIND OBJECT  ========== *
 *         Juan Manuel PEREZ RUA       *
 *        University of Burgundy       *
 * =================================== *
 * * * * * * * * * * * * * * * * * * * */
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <PatternDetector.hpp>
#include <Tracker.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ObjectFinder{
public:
    ObjectFinder();

    void applyAction();
    void stopRobot();
private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
	ros::Publisher vel_pub_;
    image_transport::Publisher ima_pub_;
    image_transport::ImageTransport *it;

    MoveBaseClient *ac;
    tf::TransformListener tf_;

    geometry_msgs::Twist curVel;
    tf::StampedTransform currPose;

    bool nav_busy;

    void goalDone(const actionlib::SimpleClientGoalState &state);
};

ObjectFinder::ObjectFinder(){
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pose_pub_ = nh_.advertise<move_base_msgs::MoveBaseGoal>("/target_object_pose", 1);
    it = new image_transport::ImageTransport(nh_);
    ima_pub_ = it->advertise("/object_image", 1);
    ac = new MoveBaseClient("move_base", true);
    tf_.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(2.0));

    nav_busy=false;

    srand (time(NULL));
}

void ObjectFinder::applyAction(  ){
    cv::VideoCapture capture(0);
    cv::Mat image;

    // image publishing
    cv_bridge::CvImagePtr frame;
    frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    // object detector
    CameraCalibration calibration(1379.9397187316185f, 1403.6016242701958f, 313.61301773339352f, 200.39306393520991f);
    std::vector<cv::Mat> patternImages;
    PatternDetector patternDetector;
    std::vector<Pattern>patterns;
    std::string defaultName = "/home/juanma/groovy_workspace/sandbox/findObject/data/duke.jpg";
    cv::Mat patternImage = cv::imread(defaultName);
    patternImages.push_back(patternImage);
    patternDetector.buildPatternsFromImages(patternImages, patterns);
    patternDetector.train(patterns);
    PatternTrackingInfo patternInfo;

    Tracker tr;
    while (ros::ok()){
        // get current position in map
        tf_.lookupTransform("/map", "/base_link", ros::Time(0), currPose);

        // get images from cam
        capture>>image;

        // object detection/tracking
        if (tr.started){
            cv::RotatedRect r = tr.track(image);
            Point2f vertices[4];
            r.points(vertices);
            for (int i = 0; i < 4; i++)
                cv::line(image, vertices[i], vertices[(i+1)%4], Scalar(128,90,30));
        }

        bool patternFound = patternDetector.findPattern(image, patternInfo);
        if (patternFound){
            patternInfo.computePose(patterns[patternInfo.patternIdx], calibration);
            patternInfo.draw2dContour(image, cv::Scalar(0,255,0));
            patternInfo.draw2dPoints(image, cv::Scalar(255,0,255));

            Rect win = patternInfo.getRect(image);
            cv::rectangle(image, win, cv::Scalar(255,0,0));

            tr.SetTracker(image, win);

            // publish
            geometry_msgs::Pose pose;
            pose.orientation.w = currPose.getRotation().w();
            pose.orientation.y = currPose.getRotation().y();
            pose.orientation.x = currPose.getRotation().x();
            pose.orientation.z = currPose.getRotation().z();
            pose.position.x = currPose.getOrigin().x();
            pose.position.y = currPose.getOrigin().y();
            pose.position.z = currPose.getOrigin().z();
            move_base_msgs::MoveBaseGoal full_pose;
            full_pose.target_pose.header.frame_id = "map";
            full_pose.target_pose.header.stamp = ros::Time::now();
            full_pose.target_pose.pose = pose;
            pose_pub_.publish(full_pose);
            // also, stop the navigation by sending the current pose
            stopRobot();
        }else{
            if (!nav_busy){
                //go randomly to a position
                float rangle = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/M_PI));
                geometry_msgs::Quaternion targetQuaternion = tf::createQuaternionMsgFromYaw(rangle);
                geometry_msgs::Point targetPoint;
                targetPoint.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2));;
                targetPoint.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2));;
                targetPoint.z = 0.0f;
                geometry_msgs::Pose targetPose;
                targetPose.orientation = targetQuaternion;
                targetPose.position = targetPoint;

                move_base_msgs::MoveBaseGoal rgoal;
                rgoal.target_pose.header.frame_id = "map";
                rgoal.target_pose.header.stamp = ros::Time::now();
                rgoal.target_pose.pose = targetPose;
                ac->sendGoal(rgoal, boost::bind(&ObjectFinder::goalDone, this, _1));
                nav_busy=true;
            }
        }

        // image publishing
        frame->header.stamp = ros::Time::now();
        frame->image = image.clone();
        ima_pub_.publish( frame->toImageMsg() );

        //show
        //cv::imshow("IMAGE", image);

        // control
        int key = cv::waitKey(30);
        if (key=='q') break;
    }
}

void ObjectFinder::goalDone(const actionlib::SimpleClientGoalState &state){

    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        std::cout<<"Goal Executed Well."<<std::endl;
    else
        std::cout<<"Goal Executed Badly."<<std::endl;

    nav_busy = false;
}

void ObjectFinder::stopRobot(){
    geometry_msgs::Pose pose;
    pose.orientation.w = currPose.getRotation().w();
    pose.orientation.y = currPose.getRotation().y();
    pose.orientation.x = currPose.getRotation().x();
    pose.orientation.z = currPose.getRotation().z();
    pose.position.x = currPose.getOrigin().x();
    pose.position.y = currPose.getOrigin().y();
    pose.position.z = currPose.getOrigin().z();
    move_base_msgs::MoveBaseGoal full_pose;
    full_pose.target_pose.header.frame_id = "map";
    full_pose.target_pose.header.stamp = ros::Time::now();
    full_pose.target_pose.pose = pose;
    ac->sendGoal(full_pose, boost::bind(&ObjectFinder::goalDone, this, _1));
    if (ros::ok()){
        for (int i=1; i<50; i++){
            curVel.angular.z = 0.0;
            curVel.linear.x  = 0.0;
            vel_pub_.publish(curVel);
            ros::spinOnce();
        }
    }
}

int main(int argc, char** argv){
    // Initialize classes and configurations
    ros::init(argc, argv, "findObject");
    ObjectFinder f;
    f.applyAction();
	return 0;
}
