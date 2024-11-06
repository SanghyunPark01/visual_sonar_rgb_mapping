/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/

#ifndef MAP_H
#define MAP_H

#include "visual_sonar_rgb_mapping/utility.h"

class Map
{
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _mptrRawPCMapData;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _mptrColoredPCMapData;
public:
    Map(void)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmpRaw(new pcl::PointCloud<pcl::PointXYZI>());
        _mptrRawPCMapData = tmpRaw;
        _mptrRawPCMapData->header.frame_id = "world";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpColored(new pcl::PointCloud<pcl::PointXYZRGB>());
        _mptrColoredPCMapData = tmpColored;
        _mptrColoredPCMapData->header.frame_id = "world";
    }
    void addMapPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr MapPoints);
    void addColoredMapPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapPoints);

    int getRawSize(void);
    int getColoredSize(void);
    sensor_msgs::PointCloud2 getAllRawCloud(void);
    sensor_msgs::PointCloud2 getAllColoredCloud(void);
    void saveMap(std::string FilePath);
};

#endif MAP_H