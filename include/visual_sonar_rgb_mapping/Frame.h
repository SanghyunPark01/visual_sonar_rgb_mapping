/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/

#ifndef FRAME
#define FRAME

#include "visual_sonar_rgb_mapping/utility.h"
#include "visual_sonar_rgb_mapping/MapPoint.h"

class Frame
{
private:
    // data
    double _mdTimeStamp;
    nav_msgs::Odometry _mOdom;

    // image
    cv::Mat _micRawImg;
    cv::Mat _micProcessedImg;
    cv::Mat _micProjectedImg;
    bool _mbProjected = false;

    // point
    std::vector<MapPoint*> _mvpMapPoint;

public:
    Frame(){_mvpMapPoint.clear();}

    //
    void processImage(void);
    void release(void);
    void visImage(void);

    //
    void setRawImg(cv::Mat Image);
    void setProjectedImg(cv::Mat Image);
    void setOdom(nav_msgs::Odometry CurrOdom);
    void pushMapPoint(MapPoint* ptrMapPoint);

    //
    double getTimeStamp(void){return _mdTimeStamp;}
    cv::Mat getRawImg(void){return _micRawImg.clone();}
    cv::Mat getProcessedImg(void){return _micProcessedImg.clone();}
    cv::Mat getProjectedImg(void){return _micProjectedImg.clone();}
    int getMapPointsSize(void){return _mvpMapPoint.size();}
    cv::Mat get_T_w_b(void){return odom2TfMat(_mOdom);}
    std::vector<cv::Point3f> getMapPointsPos(void);
};

#endif FRAME