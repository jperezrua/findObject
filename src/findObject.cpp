/* * * * * * * * * * * * * * * * * * * *
 * ==========  FIND OBJECT  ========== *
 *         Juan Manuel PEREZ RUA       *
 *        University of Burgundy       *
 * =================================== *
 * * * * * * * * * * * * * * * * * * * */
#include <iostream>
#include <ros/ros.h>
#include "_nodeSM.hpp"

int main(int argc, char** argv){
    // Initialize classes and configurations
    ros::init(argc, argv, "findObject");
    ObjectFinder f;
    f.applyAction();
    return 0;
}
