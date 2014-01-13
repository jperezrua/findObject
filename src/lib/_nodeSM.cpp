#include "_nodeSM.hpp"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

static double GOAL_DISTANCE = 0.5;

static double angle( Point pt1, Point pt2, Point pt0 ){
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static double distance( Point pt1, Point pt2 ){
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return dx*dx+dy*dy;
}

struct espx_sorter {
    bool operator() (cv::Point2f pt1, cv::Point2f pt2) { return (pt1.x < pt2.x);}
} esp_sortx;
struct espy_sorteru {
    bool operator() (cv::Point2f pt1, cv::Point2f pt2) { return (pt1.y > pt2.y);}
} esp_sortyu;
struct espy_sorterd {
    bool operator() (cv::Point2f pt1, cv::Point2f pt2) { return (pt1.y < pt2.y);}
} esp_sortyd;

ObjectFinder::ObjectFinder(){
    _CURRENT_STATE = _DEFAULT;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    diff_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/diff_pose",1);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/object_pose",1);
    gopose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/go_pose",1);
    mapSub = nh_.subscribe("/map",10,&ObjectFinder::mapper,this);
    //occSub = nh_.subscribe("/move_base/local_costmap/inflated_obstacles",10,&ObjectFinder::mapperobs,this);

    nh_.param<std::string>("/findObject/template_name", template_name, "/home/roboticslab/groovy_workspace/sandbox/findObject/data/qr.jpg");
    nh_.param<std::string>("/findObject/kinect_frame_name", kinect_frame_name, "/camera_depth_optical_frame");
    nh_.param<std::string>("/findObject/robot_frame_name", fixed_frame, "/map");
    nh_.param<std::string>("/findObject/depth_node_name", depth_node_name, "/camera/depth/image_raw");
    //nh_.param<std::string>("/findObject/rgb_node_name", rgb_node_name, "/img_comp");
    nh_.param<std::string>("/findObject/rgb_node_name", rgb_node_name, "/camera/rgb/image_raw");
    nh_.param<std::string>("/findObject/caminfo_node_name", caminfo_node_name, "/camera/rgb/camera_info");

    ac = new MoveBaseClient("move_base", true);

    templ = imread(template_name.c_str());

    it = new image_transport::ImageTransport(nh_);
    ima_sub_ = it->subscribe(rgb_node_name, 1,&ObjectFinder::readImage,this);
    dep_sub_ = it->subscribe(depth_node_name, 1,&ObjectFinder::readDepth,this);
    cam_info_ =  nh_.subscribe(caminfo_node_name, 1, &ObjectFinder::readKam, this);
    ima_pub_ = it->advertise("/object_image", 1);
    im_ready=false;
    dep_ready=false;
    kam_ready=false;
    mapfready=false;

    init_point = cv::Point(77, 85);
    srand (time(NULL));

    // wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))) ROS_INFO("Waiting for the move_base action server to come up");
    moving=false;
    currPathIdx = 0;
    firsttime = true;
    yawAnglePose=2*M_PI/10;
    findops=0;
}

static int map_height, map_width, map_origin_x, map_origin_y;
static double map_resolution;

void ObjectFinder::mapper(const nav_msgs::OccupancyGridPtr& map){
    map_height=map->info.height;
    map_width =map->info.width;
    map_resolution=map->info.resolution;
    map_origin_x=map->info.origin.position.x;
    map_origin_y=map->info.origin.position.y;

    char *vec;
    vec = new char[map->data.size()];
    for (size_t i=0; i<map->data.size(); i++){
        vec[i]=map->data[i];
    }

    cv::Mat mapa = cv::Mat(map_height, map_width, CV_8S, vec).clone();
    mapf = cv::Mat::zeros(map_height, map_width, CV_8U);

    //Convert map info in two arrays: explored and occupacy
    for (int i=0; i<mapa.rows; i++)
        for (int j=0; j<mapa.cols; j++){
            if (int(mapa.at<char>(i,j))==-1){
                mapf.at<uchar>(i,j)=128;
            }else if(int(mapa.at<char>(i,j))>0){
                mapf.at<uchar>(i,j)=0;
            }else{
                mapf.at<uchar>(i,j)=255;
            }
        }

    cv::Mat map_proc=mapf.clone();
    cv::Mat color_map;
    cv::cvtColor(map_proc, color_map, CV_GRAY2BGR);
    mapfready=true;
    mapSub.shutdown();
    cv::circle(color_map, init_point, 2, cv::Scalar(0,255,0), 2);
    cv::imshow("map", color_map);

    delete [] vec;
}

double ObjectFinder::findObjectYaw(const cv::Mat &depth, const cv::Mat &mask,
                                   double alpha_x, double u0, const cv::Rect &rec){
    double yaw=0.0;

    double px=rec.x;
    cv::Mat zvals;
    depth.convertTo(zvals, CV_64FC1);

    int N = int(cv::sum(mask)[0]/255);

    cv::Mat X = cv::Mat::ones(N,2,CV_64FC1);
    cv::Mat Z = cv::Mat::ones(N,1,CV_64FC1);
    int cont=0;
    for (int i=0; i<depth.rows; i++){
        for (int j=0; j<depth.cols; j++){
            double zval = zvals.at<double>(i,j);
            int masked = mask.at<uchar>(i,j);

            if (masked!=0){
                X.at<double>(cont,0) = (j+px-u0)*zval/alpha_x;
                Z.at<double>(cont,0) = zval;
                cont++;
            }
        }
    }
    X = X - *std::min_element(X.begin<double>(), X.end<double>());
    cv::Mat B = (X.t()*X).inv()*X.t()*Z;

    cv::Mat Z2 = B.at<double>(0,0)*X.col(0)+B.at<double>(1,0);

    yaw = std::atan2( (Z2.at<double>(N-1,0)-Z2.at<double>(0,0)),(X.at<double>(N-1,0)-X.at<double>(0,0)) );

    return yaw;
}

geometry_msgs::Quaternion ObjectFinder::getRotMat(CameraCalibration cc, cv::Size sz, std::vector<Point2f> points2d){

    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux,taux;

    std::sort(points2d.begin(), points2d.end(), esp_sortx); //organizing by x component
    std::sort(points2d.begin(), points2d.begin()+1, esp_sortyu); //organizing by y (the first two): the biggest y is the first
    std::sort(points2d.end()-1, points2d.end(), esp_sortyd); //organizing the last two: the biggest y is last

    // Normalized dimensions:
    const float maxSize = std::max(sz.height, sz.width);
    const float unitW = sz.width / maxSize;
    const float unitH = sz.height / maxSize;
    std::vector<Point3f> points3d(4);
    points3d[0] = cv::Point3f(-unitW, -unitH, 0);
    points3d[1] = cv::Point3f(-unitW, unitH, 0);
    points3d[2] = cv::Point3f(unitW, unitH, 0);
    points3d[3] = cv::Point3f(unitW, -unitH, 0);
    cv::solvePnP(points3d, points2d, cc.getIntrinsic(), cc.getDistorsion(),raux,taux);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);

    cv::Mat_<float> rotMat(3,3);
    cv::Rodrigues(Rvec, rotMat);
    tf::Quaternion q;
    //q.setRotation(tf::Vector3(0,0,1), acos((rotMat(0,0)+rotMat(1,1))/2));
    q.setRotation(tf::Vector3(0,0,1), acos(rotMat(0,0)));
    geometry_msgs::Quaternion qt;
    quaternionTFToMsg(q,qt);
    return qt;
}

Rect ObjectFinder::getBB(std::vector<cv::Point>  obj){
    Point ini, end;
    ini.x=ini.y=10000;
    end.x=end.y=0;
    for ( size_t j = 0; j < obj.size(); j++ ){
        ini.x = std::min(ini.x, obj[j].x);
        ini.y = std::min(ini.y, obj[j].y);
        end.x = std::max(end.x, obj[j].x);
        end.y = std::max(end.y, obj[j].y);
    }
    return Rect(ini, end);
}

std::vector<cv::Point> ObjectFinder::getMostSimilObj(std::vector<std::vector<Point> > squares, const cv::Mat I){
    std::vector<cv::Point>  out;
    if (squares.size()>0){
        float maxd=0.0;
        int maxii=0.0;
        for( size_t i = 0; i < squares.size(); i++ ){
            float dis=0.0;
            std::vector<cv::Point> objectCoor = squares[i];
            Rect r=getBB(objectCoor);
            cv::Mat roi( I.clone(), r );
            cvtColor( roi, roi, CV_BGR2HSV );
            cvtColor( templ, templ, CV_BGR2HSV );

            int h_bins = 30; int s_bins = 30;
            int histSize[] = { h_bins, s_bins };
            float h_ranges[] = { 0, 256 }, s_ranges[] = { 0, 180 };
            const float* ranges[] = { h_ranges, s_ranges };
            int channels[] = { 0, 1 };

            MatND hist_base, hist_test;
            calcHist( &templ, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
            normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
            calcHist( &roi, 1, channels, Mat(), hist_test, 2, histSize, ranges, true, false );
            normalize( hist_test, hist_test, 0, 1, NORM_MINMAX, -1, Mat() );

            dis = compareHist( hist_base, hist_test, CV_COMP_CHISQR );
            if (dis>0.375) squares[i].clear(); //if too disimilar, eliminate

            if (dis>maxd){
                maxd=dis;
                maxii=i;
            }
        }
        out=squares[maxii];
    }
    return out;
}

void ObjectFinder::detectObject(const cv::Mat& I, std::vector<cv::Point>& objectCoor){

     Mat occludedSquare = I.clone();

     Mat occludedSquare8u;
     cvtColor(occludedSquare, occludedSquare8u, CV_BGR2GRAY);

     Mat thresh;
     adaptiveThreshold(occludedSquare8u, thresh, 255, ADAPTIVE_THRESH_MEAN_C ,
                       THRESH_BINARY_INV, 3, 10);

     std::vector<std::vector<Point> > contours;
     findContours(thresh, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
     std::vector<Point> approx;

     std::vector<std::vector<Point> > squares;

     for( size_t i = 0; i < contours.size(); i++ ){
         approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
         if( approx.size() == 4 &&
             fabs(contourArea(Mat(approx))) > 150 &&
             fabs(contourArea(Mat(approx))) < 22000 &&
             isContourConvex(Mat(approx)) ){
             double maxCosine = 0;

             for( int j = 2; j < 5; j++ ){
                 double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                 maxCosine = MAX(maxCosine, cosine);
             }
             double edgeratio = (distance(approx[0],approx[1])+distance(approx[2],approx[3]))
                                /(distance(approx[1],approx[2])+distance(approx[3],approx[0]));

             if( maxCosine < 0.25 && edgeratio>0.35 && edgeratio<1.65)
                 squares.push_back(approx);
         }
     }

     objectCoor = getMostSimilObj(squares, I);
}

void ObjectFinder::applyAction(  ){
    cv::Mat image_use; // rgb image buffer
    cv::Mat image; // rgb image buffer
    cv::Mat depth; // depth image buffer
    std::vector<cv::Point> lastCoor; // detected object: coordinates in image frame (2d)
    float Zobj=0.0; // object depth value
    cv::Point p;
    cv::Mat groi, gmask;

    cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >(); // bridged image pointer for publishing
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    std::cout<<"STARTING STATE MACHINE!..."<<std::endl;

    while (ros::ok()){

        if (im_ready){
            image=rgb_im.clone(); //drawing
            image_use=image.clone(); //process

            im_ready=false;

            switch (_CURRENT_STATE){

            case _DEFAULT:
            case _DIFF_POSE_NOT_REACHED:
            case _OBJECT_NOT_FOUND:
            case _TARGET_NOT_REACHABLE:
            case _ROBUST_OBJECT_NOT_FOUND:
                // EXPLORE: DIFFERENTIAL MOTION
            {
                std::cout<<"DIFF MOTION"<<std::endl;
                if (yawAnglePose==0 || firsttime){
                    if (currPathIdx>=pathGraph.size() || firsttime){
                        pather();
                        currPathIdx = 0;
                    }

                    p = pathGraph[currPathIdx];
                    currPathIdx++;
                }else{
                    if (currPathIdx>=pathGraph.size()){
                        yawAnglePose=0;
                        break;
                    }else{
                        p = pathGraph[currPathIdx];
                    }
                }

                geometry_msgs::Pose_< std::allocator<void> > pos;
                pos.position.x = p.x*map_resolution+map_origin_x;
                pos.position.y = p.y*map_resolution+map_origin_y;
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id=fixed_frame;
                pose.header.stamp = ros::Time::now();
                pose.pose = pos;
                geometry_msgs::Quaternion qt;
                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yawAnglePose),qt);
                yawAnglePose=yawAnglePose+2*M_PI/10;
                if (yawAnglePose>=2*M_PI) yawAnglePose=0;
                pose.pose.orientation = qt;
                diff_pub_.publish(pose);

                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = fixed_frame;
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose = pose;

                targetReached=false;
                ac->sendGoal(goal, boost::bind(&ObjectFinder::goalDone, this, _1));
                moving=true;

                _CURRENT_STATE = _WAITING_POSE;
            }break;

            case _WAITING_POSE:
            {
                if (!moving){
                    if (targetReached)
                        _CURRENT_STATE = _DIFF_POSE_REACHED;
                    else
                        _CURRENT_STATE = _DIFF_POSE_NOT_REACHED;
                }else{
                    _CURRENT_STATE = _DIFF_POSE_REACHED;
                }
            }break;

            case _DIFF_POSE_REACHED:
                // SEARCH ACTION
            {
                std::cout<<"SEARCH OBJECT"<<std::endl;
                std::vector<cv::Point> objectCoor;
                detectObject(image, objectCoor);
                if (dep_ready && objectCoor.size()>0){
                    cv::normalize(dep_im, depth, 0, 255, NORM_MINMAX);
                    depth.convertTo(depth, CV_8UC1);

                    // objected detected: compute object depth
                    Rect r=getBB(objectCoor);
                    cv::Mat roi( dep_im, r );
                    groi=roi.clone();
                    cv::Mat mask( depth, r );
                    cv::threshold(mask, mask, 5, 255, THRESH_BINARY); // omit bas depth readings
                    gmask=mask.clone();
                    Zobj = mean(roi, mask)[0]/1000; //apparently there is aproblem  with gazebo (units mm or m?)
                    //Zobj = mean(roi, mask)[0]; //apparently there is aproblem  with gazebo (units mm or m?)

                    const Point* po = &objectCoor[0];
                    int n = 4;
                    polylines(image, &po, &n, 1, true, Scalar(0,255,0), 3);

                    lastCoor = objectCoor;

                    init_point = p; // update init_point with the point that last saw the object
                    _CURRENT_STATE = _OBJECT_FOUND;
                    if (moving){
                        moving=false; //Its actually moving, but i want to send the next goal safely
                    }
                }else{
                    if (findops>=5){
                        findops=0;
                        if (moving){
                            _CURRENT_STATE = _WAITING_POSE;
                        }else{
                            _CURRENT_STATE = _OBJECT_NOT_FOUND;
                        }
                    }else{
                        findops++;
                        _CURRENT_STATE = _DIFF_POSE_REACHED;
                    }
                }
            }break;

            case _OBJECT_FOUND:
                // IN-FRONT MOTION
                std::cout<<"IN FRON MOTION"<<std::endl;
                if (kam_ready){
                    kam_ready=false;

                    cv::Rect r=getBB(lastCoor);
                    float u = r.x + r.width/2, v = r.y + r.height/2;

                    float alpha_x=kam.at(0), alpha_y = kam.at(4);
                    float uo = kam.at(2), vo = kam.at(5);
                    // compute full 3D coordinates of the object (based on calibration)
                    float Xobj = (u-uo)*Zobj/alpha_x;
                    float Yobj = (v-vo)*Zobj/alpha_y;
                    // now estimate rotation Quaternion
                    cv::circle(image, cv::Point(u,v), 5, Scalar(0,255,255),2);

                    double yaw = findObjectYaw(groi, gmask, alpha_x, uo, r);
                    //std::cout<<"YAW: "<<yaw*180/M_PI<<std::endl;

                    // find object pose in camera link
                    tf::Vector3 point(Xobj,Yobj,Zobj);
                    geometry_msgs::Pose_< std::allocator<void> > pos;
                    pos.position.x = point.z();
                    pos.position.y = -point.x();
                    pos.position.z = -point.y();
                    pos.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id="/camera_link";
                    pose.header.stamp = ros::Time::now();
                    pose.pose = pos;
                    pose_pub_.publish(pose);

                    // find pose to reach in camera link
                    geometry_msgs::PoseStamped gopose;
                    gopose=pose;
                    gopose.pose.position.x = pose.pose.position.x - GOAL_DISTANCE*cos(yaw);
                    gopose.pose.position.y = pose.pose.position.y - GOAL_DISTANCE*sin(yaw);
                    gopose_pub_.publish(gopose);

                    // send goal to ac (he will solve the frame)
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = gopose.header.frame_id;
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = gopose.pose;

                    targetReached=false;
                    ac->sendGoal(goal, boost::bind(&ObjectFinder::goalDone, this, _1));
                    moving=true;

                    _CURRENT_STATE = _WAITING_TARGET;
                    //_CURRENT_STATE = _DEFAULT;
                }
                break;

            case _WAITING_TARGET:
                // WAIT FOR ACTION TO COMPLETE
                if (!moving){
                    if (targetReached)
                        _CURRENT_STATE = _TARGET_REACHED;
                    else
                        _CURRENT_STATE = _TARGET_NOT_REACHABLE;
                }
                break;

            case _TARGET_REACHED:
                // ROBUST SEARCH
                std::cout<<"ROBUST SEARCH"<<std::endl;
                /*{
                    std::vector<cv::Mat> patternImages;
                    PatternDetector patternDetector;
                    std::vector<Pattern> patterns;
                    CameraCalibration cc(kam.at(0), kam.at(4), kam.at(2), kam.at(5));
                    cv::Mat patternImage = templ.clone();
                    patternImages.push_back(patternImage);
                    patternDetector.buildPatternsFromImages(patternImages, patterns);
                    patternDetector.train(patterns);
                    PatternTrackingInfo patternInfo;

                    bool patternFound = patternDetector.findPattern(image_use, patternInfo);

                    if (patternFound){
                        patternInfo.draw2dContour(image, cv::Scalar(0,255,255));
                        _CURRENT_STATE = _ROBUST_OBJECT_FOUND;
                    }else{
                        _CURRENT_STATE = _ROBUST_OBJECT_NOT_FOUND;
                    }
                }*/
                break;

            case _ROBUST_OBJECT_FOUND:
                // STOP
                std::cout<<"STOP"<<std::endl;
                break;

            }

            // publish image
            image.copyTo(frame->image);
            frame->header.stamp = ros::Time::now();
            ima_pub_.publish(frame->toImageMsg());


            firsttime = false;
        }

        // spin, just once
        ros::spinOnce();

        // wait
        cv::waitKey(2);
    }

}

void ObjectFinder::goalDone(const actionlib::SimpleClientGoalState &state){
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        targetReached=true;
    else
        targetReached=false;
    moving=false;
}

void ObjectFinder::readImage(const sensor_msgs::ImageConstPtr& kinectImage){
    cv::Mat im = cv_bridge::toCvShare(kinectImage, "bgr8")->image;
    if (!im.empty()){
        im.copyTo(rgb_im);
        im_ready=true;
    }
}
void ObjectFinder::readDepth(const sensor_msgs::ImageConstPtr& kinectImage){
    cv::Mat im = cv_bridge::toCvShare(kinectImage)->image;
    if (!im.empty()){
        im.copyTo(dep_im);
        dep_ready=true;
    }
}

void ObjectFinder::readKam(const sensor_msgs::CameraInfoConstPtr &camInfo){
    kam = camInfo->K;
    kam_ready = true;
}

void ObjectFinder::pather()
{
    cv::Mat map_proc=mapf.clone();
    cv::Mat color_map;
    cv::cvtColor(map_proc, color_map, CV_GRAY2BGR);

    // Generate N random points within the map
    int cont_valid_numbers=0;
    const int N=4;
    pathGraph.clear();
    pathGraph.push_back(init_point);

    while(cont_valid_numbers<N){

        double dx = init_point.x - 10.0 + ( (double)rand() / RAND_MAX )*20;
        double dy = init_point.y -10.0 + ( (double)rand() / RAND_MAX )*20;
        //double dx = init_point.x;
        //double dy = init_point.y;

        int px = dx;
        int py = dy;
        cv::Point p = cv::Point(py,px);

        if (mapf.at<uchar>(p.x,p.y)==255){
            cont_valid_numbers++;
            pathGraph.push_back(p);
            cv::circle(color_map, p, 1, cv::Scalar(0,255,0), 2);
        }//
    }

    cv::imshow("map", color_map);
}
