/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/
#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "visual_sonar_rgb_mapping/utility.h"
#include "visual_sonar_rgb_mapping/Map.h"
#include "visual_sonar_rgb_mapping/MapPoint.h"
#include "visual_sonar_rgb_mapping/Frame.h"

struct AssociatedSonarOdomData
{
    nav_msgs::Odometry prevOdom;
    sensor_msgs::LaserScan SonarData;
    nav_msgs::Odometry currOdom;
    AssociatedSonarOdomData(nav_msgs::Odometry prev_odom, sensor_msgs::LaserScan sonar_data, nav_msgs::Odometry curr_odom):
    prevOdom(prev_odom),SonarData(sonar_data),currOdom(curr_odom){}
    AssociatedSonarOdomData(){}
};

class MapDrawer
{
private:
/******* Variables *******/
    // Param
    std::string SONAR_TOPIC, ODOM_TOPIC, IMG_TOPIC;
    int MAX_QUEUE_SIZE;
    std::vector<double> EXTRINSIC_BODY_SONAR_VEC;
    std::vector<double> EXTRINSIC_RVIZ_WORLD_VEC;
    Eigen::Matrix4d EXTRINSIC_RVIZ_WORLD;
    std::vector<double> EXTRINSIC_BODY_CAM_VEC;
    std::vector<double> INTRINSIC_CAM_VEC;
    std::vector<double> CAM_DISTORTION_COEF_VEC;
    cv::Mat INTRINSIC_CAM_MAT = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat CAM_DISTORTION_COEF_MAT = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat EXTRINSIC_BODY_CAM_MAT = cv::Mat::zeros(4, 4, CV_64F);
    float SONAR_ANGLE_MIN, SONAR_ANGLE_MAX;
    double MIN_INTENSITY_THD;
    bool PUBLISH;
    bool SAVE_MAP;
    bool VIS_IMG;
    bool LOGGING;

    // ROS
    ros::NodeHandle _nh;
    ros::Subscriber _mSubSonar;
    ros::Subscriber _mSubOdom;
    ros::Subscriber _mSubImg;
    tf2_ros::TransformBroadcaster _mBr;
    ros::Publisher _mPubInterOdom;
    ros::Publisher _mPubCurrPC;
    ros::Publisher _mPubRawMap;
    ros::Publisher _mPubColoredMap;

    // ROS Data
    geometry_msgs::TransformStamped _mTransformStamped;
    std::queue<sensor_msgs::LaserScan> _mqSonarBuf;
    std::queue<nav_msgs::Odometry> _mqOdomBuf;
    std::queue<nav_msgs::Odometry> _mqOdomBuf2;
    std::queue<sensor_msgs::Image> _mqImgBuf;
    std::mutex _mtxCallbackSonar;
    std::mutex _mtxCallbackOdom;
    std::mutex _mtxCallbackOdom2;
    std::mutex _mtxCallbackImg;
    double _mdPrevSonarCallbackStamp = 0;
    double _mdPrevOdomCallbackStamp = 0;
    double _mdPrevImgCallbackStamp = 0;

    // Process Data
    double _mdPrevOdomStamp = 0;
    nav_msgs::Odometry _mPrevOdom;
    double _mdPrevOdomStamp2 = 0;
    nav_msgs::Odometry _mPrevOdom2;

    // Mapping
    Map* _mptrSonarMap;
    Eigen::Matrix4d _mT_b_s;
    cv::Mat _mT_c_b;

    // RGB
    std::queue<Frame*> _mqSyncFrameBuf;
    std::queue<MapPoint*> _mqRawMapPointBuf;
    std::mutex _mtxSyncFrame;
    std::mutex _mtxRawMapPoint;

    // Debuging
    int _mnDropSonarData = 0;
    int _mnDropOdomData = 0;
    int _mnDropImgData = 0;
    int _mnInterpolateErrorNum = 0;
    int _mnSonarScanNum = 0;

/******* Function *******/
    // init
    void checkParam(void);
    void sendRvizTF(void);
    void sendRvizTF(double Time);

    // ROS Callback
    void callbackSonar(const sensor_msgs::LaserScan &msgSonar);
    void callbackOdom(const nav_msgs::Odometry &msgOdom);
    void callbackImg(const sensor_msgs::ImageConstPtr &msgImg);

    // Process Data
    bool matchData(AssociatedSonarOdomData &Data);
    AssociatedSonarOdomData interpolateOdom(AssociatedSonarOdomData Data);

    // Mapping
    bool refineData(AssociatedSonarOdomData &Data);
    void addRawMapPoint(AssociatedSonarOdomData Data);

    // RGB
    bool syncImg(void);
    bool accumRawMapPoints(Frame* &ptrCurrFrame);
    void renderMapPoints(Frame* &ptrCurrFrame);
    void projectMapPoints(Frame* &ptrFrame, std::vector<cv::Point3f> &MapPoints, std::vector<cv::Point2f> &ProjectedPoints);

public:
    MapDrawer(const ros::NodeHandle& nh_);
    void processData(void);
    void buildMap(void);
    void debugging(void);
    void saveMap(void);
};

#endif MAPDRAWER_H