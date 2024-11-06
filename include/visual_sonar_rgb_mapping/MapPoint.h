/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/

#ifndef MAPPOINT
#define MAPPOINT

#include "visual_sonar_rgb_mapping/utility.h"

class MapPoint
{
private:
    double _mdTimeStamp;
    cv::Point3f _mpPosition; // world frame
    int _mR;
    int _mG;
    int _mB;
public:
    MapPoint(){}
    
    // set
    void setTimeStamp(double TimeStamp);
    void setPosition(float x, float y, float z);

    // get
    double getTimeStamp(void){return _mdTimeStamp;}
    cv::Point3f getPosition(void){return _mpPosition;}
};

#endif MAPPOINT