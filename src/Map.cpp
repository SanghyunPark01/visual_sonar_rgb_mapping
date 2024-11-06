/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/
#include "visual_sonar_rgb_mapping/Map.h"

void Map::addMapPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr MapPoints)
{
    MapPoints->header.frame_id = "world";
    *_mptrRawPCMapData += *MapPoints;
}
void Map::addColoredMapPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapPoints)
{
    MapPoints->header.frame_id = "world";
    *_mptrColoredPCMapData += *MapPoints;
}
int Map::getRawSize(void)
{
    return _mptrRawPCMapData->size();
}
int Map::getColoredSize(void)
{
    return _mptrColoredPCMapData->size();
}
sensor_msgs::PointCloud2 Map::getAllRawCloud(void)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*_mptrRawPCMapData, output);
    return output;
}
sensor_msgs::PointCloud2 Map::getAllColoredCloud(void)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*_mptrColoredPCMapData, output);
    output.header.frame_id = "world";
    return output;
}
void Map::saveMap(std::string FilePath)
{
    pcl::PCDWriter pcdWriter;
    pcdWriter.writeBinary(FilePath, *_mptrColoredPCMapData);
}