/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/

#include "visual_sonar_rgb_mapping/Frame.h"

void Frame::processImage(void)
{
    cv::Mat FilteredImg;

        // Histogram Equalization
    cv::Mat img_ycrcb;
    cv::cvtColor(_micRawImg, img_ycrcb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> channels;
    cv::split(img_ycrcb, channels); 
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->apply(channels[0], channels[0]);
    cv::merge(channels, img_ycrcb);
    cv::cvtColor(img_ycrcb, FilteredImg, cv::COLOR_YCrCb2BGR);
    img_ycrcb.release();

    _micProcessedImg = FilteredImg.clone();
}
void Frame::release(void)
{
    _micRawImg.release();
    _micProcessedImg.release();
    _micProjectedImg.release();

    for(int i = 0; i < _mvpMapPoint.size(); i++)
    {
        delete _mvpMapPoint[i];
    }
}
void Frame::visImage(void)
{
    if(_mbProjected) cv::imshow("frame",_micProjectedImg);
    else cv::imshow("frame",_micProcessedImg);
    cv::waitKey(1);
}

void Frame::setRawImg(cv::Mat Image)
{
    _micRawImg = Image.clone();
    processImage();
}
void Frame::setProjectedImg(cv::Mat Image)
{
    _micProjectedImg = Image;
    _mbProjected = true;
}

void Frame::setOdom(nav_msgs::Odometry CurrOdom)
{
    _mOdom = CurrOdom;
    _mdTimeStamp = CurrOdom.header.stamp.toSec();
}
void Frame::pushMapPoint(MapPoint* ptrMapPoint)
{
    _mvpMapPoint.push_back(ptrMapPoint);
}
std::vector<cv::Point3f> Frame::getMapPointsPos(void)
{
    std::vector<cv::Point3f> MapPointsPos;
    for(int i = 0; i < _mvpMapPoint.size(); i++)
    {
        MapPointsPos.push_back(_mvpMapPoint[i]->getPosition());
    }
    return MapPointsPos;
}
