/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/
#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <signal.h>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <tuple>
#include <cmath>
#include <boost/filesystem.hpp>
#include <sys/resource.h>

#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>

void mySigintHandler(int sig){
    ros::shutdown();
}

void eigenToTf(const Eigen::Matrix4d& matrix, geometry_msgs::TransformStamped& transformStamped)
{
    transformStamped.transform.translation.x = matrix(0, 3);
    transformStamped.transform.translation.y = matrix(1, 3);
    transformStamped.transform.translation.z = matrix(2, 3);

    Eigen::Matrix3d rotationMatrix = matrix.block<3, 3>(0, 0);

    Eigen::Quaterniond quaternion(rotationMatrix);

    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();
}

// Linear interpolation
double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

void laserScanToPointCloud(sensor_msgs::LaserScan scanMsg, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    cloud->header.frame_id = scanMsg.header.frame_id;
    cloud->header.stamp = pcl_conversions::toPCL(scanMsg.header).stamp;

    float angle = scanMsg.angle_min;

    for (size_t i = 0; i < scanMsg.ranges.size(); ++i) {
        float range = scanMsg.ranges[i];

        if (range < scanMsg.range_min || range > scanMsg.range_max) {
            angle += scanMsg.angle_increment;
            continue;
        }

        pcl::PointXYZI point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;

        if (!scanMsg.intensities.empty()) {
            point.intensity = scanMsg.intensities[i];
        } else {
            point.intensity = 0.0;
        }

        cloud->points.push_back(point);

        angle += scanMsg.angle_increment;
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPoint(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, 
    Eigen::Matrix4d& transform_matrix) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto& point : *input_cloud) {
        pcl::PointXYZI transformed_point;
        
        Eigen::Vector4d original_point(point.x, point.y, point.z, 1.0);
        
        Eigen::Vector4d new_point = transform_matrix * original_point;
        
        transformed_point.x = new_point[0];
        transformed_point.y = new_point[1];
        transformed_point.z = new_point[2];
        transformed_point.intensity = point.intensity;  // intensity 유지

        transformed_cloud->points.push_back(transformed_point);
    }
    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1;
    transformed_cloud->is_dense = true;

    return transformed_cloud;
}

Eigen::Matrix4d odomToTransformationMatrix(const nav_msgs::Odometry& odom) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    transformation(0, 3) = odom.pose.pose.position.x;
    transformation(1, 3) = odom.pose.pose.position.y;
    transformation(2, 3) = odom.pose.pose.position.z;

    tf::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    );

    tf::Matrix3x3 tfRotation(q);
    Eigen::Matrix3d rotation;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation(i, j) = tfRotation[i][j];
        }
    }

    transformation.block<3, 3>(0, 0) = rotation;

    return transformation;
}

cv::Mat convertROSImage2CvMat(const sensor_msgs::Image& msgImg)
{
    try {
        cv_bridge::CvImagePtr ptrCvBridge = cv_bridge::toCvCopy(msgImg, sensor_msgs::image_encodings::BGR8);
        return ptrCvBridge->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(1);
        return cv::Mat();
    }
}

cv::Mat odom2TfMat(nav_msgs::Odometry odom)
{
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;

    double qx = odom.pose.pose.orientation.x;
    double qy = odom.pose.pose.orientation.y;
    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;

    cv::Mat transformationMatrix = cv::Mat::eye(4, 4, CV_64F);

    double r11 = 1 - 2 * (qy * qy + qz * qz);
    double r12 = 2 * (qx * qy - qz * qw);
    double r13 = 2 * (qx * qz + qy * qw);

    double r21 = 2 * (qx * qy + qz * qw);
    double r22 = 1 - 2 * (qx * qx + qz * qz);
    double r23 = 2 * (qy * qz - qx * qw);

    double r31 = 2 * (qx * qz - qy * qw);
    double r32 = 2 * (qy * qz + qx * qw);
    double r33 = 1 - 2 * (qx * qx + qy * qy);

    transformationMatrix.at<double>(0, 0) = r11;
    transformationMatrix.at<double>(0, 1) = r12;
    transformationMatrix.at<double>(0, 2) = r13;

    transformationMatrix.at<double>(1, 0) = r21;
    transformationMatrix.at<double>(1, 1) = r22;
    transformationMatrix.at<double>(1, 2) = r23;

    transformationMatrix.at<double>(2, 0) = r31;
    transformationMatrix.at<double>(2, 1) = r32;
    transformationMatrix.at<double>(2, 2) = r33;

    transformationMatrix.at<double>(0, 3) = x;
    transformationMatrix.at<double>(1, 3) = y;
    transformationMatrix.at<double>(2, 3) = z;

    return transformationMatrix.clone();
}

bool convertTransformMat2Vec(cv::Mat T, cv::Mat &tvec, cv::Mat &rvec)
{
    tvec = T(cv::Rect(3, 0, 1, 3)).clone();
    auto tmpR = T(cv::Rect(0, 0, 3, 3)).clone();
    cv::Rodrigues(tmpR, rvec);

    return true;
}

#endif UTILITY_H