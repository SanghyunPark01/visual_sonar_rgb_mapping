/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/

#include "visual_sonar_rgb_mapping/MapPoint.h"

void MapPoint::setTimeStamp(double TimeStamp)
{
    _mdTimeStamp = TimeStamp;
}
void MapPoint::setPosition(float x, float y, float z)
{
    _mpPosition.x = x;
    _mpPosition.y = y;
    _mpPosition.z = z;
}
