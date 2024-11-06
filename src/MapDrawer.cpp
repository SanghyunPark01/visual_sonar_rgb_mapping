/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/
#include "visual_sonar_rgb_mapping/MapDrawer.h"

MapDrawer::MapDrawer(const ros::NodeHandle& nh_):_nh(nh_)
{
    _nh.param<std::string>("/topic/sonar", SONAR_TOPIC, "/");
    _nh.param<std::string>("/topic/odom", ODOM_TOPIC, "/");
    _nh.param<std::string>("/topic/image", IMG_TOPIC, "/");

    _nh.param<int>("/topic/max_queue_size", MAX_QUEUE_SIZE, 1000);

    _nh.param<std::vector<double>>("/calibration/T_b_s", EXTRINSIC_BODY_SONAR_VEC, std::vector<double>());
    _nh.param<std::vector<double>>("/calibration/T_rviz_w", EXTRINSIC_RVIZ_WORLD_VEC, std::vector<double>());
    _nh.param<std::vector<double>>("/calibration/camera_intrinsic", INTRINSIC_CAM_VEC, std::vector<double>());
    _nh.param<std::vector<double>>("/calibration/dist_coeffs", CAM_DISTORTION_COEF_VEC, std::vector<double>());
    _nh.param<std::vector<double>>("/calibration/T_b_c", EXTRINSIC_BODY_CAM_VEC, std::vector<double>());

    _nh.param<float>("/maping/sonar_angle_min", SONAR_ANGLE_MIN, 0.0);
    _nh.param<float>("/maping/sonar_angle_max", SONAR_ANGLE_MAX, 360.0);
    _nh.param<double>("/maping/min_intensity_threshold", MIN_INTENSITY_THD, 0.0);
    _nh.param<bool>("/output/publish", PUBLISH, true);
    _nh.param<bool>("/output/save_map", SAVE_MAP, true);
    _nh.param<bool>("/output/vis_image", VIS_IMG, true);
    _nh.param<bool>("/debug/log", LOGGING, true);

    _mSubSonar = _nh.subscribe(SONAR_TOPIC, 1000, &MapDrawer::callbackSonar, this);
    _mSubOdom = _nh.subscribe(ODOM_TOPIC, 1000, &MapDrawer::callbackOdom, this);
    _mSubImg = _nh.subscribe(IMG_TOPIC, 1000, &MapDrawer::callbackImg, this);
    
    _mPubInterOdom = _nh.advertise<nav_msgs::Odometry>("/sonar_mapping/interpolated_odom", 1000);
    _mPubRawMap = _nh.advertise<sensor_msgs::PointCloud2>("/sonar_mapping/curr_scan", 1000);
    _mPubColoredMap = _nh.advertise<sensor_msgs::PointCloud2>("/sonar_mapping/colored_cloud", 1000);

    _mptrSonarMap = new Map();

    checkParam();
}
void MapDrawer::checkParam(void)
{
    std::cout << "\n";
    std::cout << "\033[1;32m    ┌────────────────────────────────────────┐\033[0m" << "\n";
    std::cout << "\033[1;32m┌───┤           SONAR RGB Mapping            ├───┐\033[0m" << "\n";
    std::cout << "\033[1;32m│   └────────────────────────────────────────┘   │\033[0m" << "\n";
    std::cout << "\033[1;32m│ Author: Sanghyun Park                          │\033[0m" << "\n";
    std::cout << "\033[1;32m│ CoCEL @POSTECH, KR                             │\033[0m" << "\n";
    std::cout << "\033[1;32m│**Contact**                                     │\033[0m" << "\n";
    std::cout << "\033[1;32m│E-mail: pash0302@gmail.com                      │\033[0m" << "\n";
    std::cout << "\033[1;32m│E-mail: pash0302@postech.ac.kr                  │\033[0m" << "\n";
    std::cout << "\033[1;32m└────────────────────────────────────────────────┘\033[0m" << "\n";
    std::cout << "\n";
    std::cout << "\033[1;33mSonar Topic: \033[0m" << SONAR_TOPIC << "\n";
    std::cout << "\033[1;33mOdom Topic: \033[0m" << ODOM_TOPIC << "\n";

    if(EXTRINSIC_BODY_SONAR_VEC.size() != 16)
    {
        ROS_ERROR("Wrong extrinsic matrix. size must be 16");
        exit(1);
    }
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            _mT_b_s(i, j) = EXTRINSIC_BODY_SONAR_VEC[i * 4 + j]; // i: row, j: col
        }
    }
    EXTRINSIC_BODY_SONAR_VEC.clear();
    std::cout << "\033[1;33mExtrinsic T_b_s: \n\033[0m";
    std::cout << _mT_b_s << "\n";


    if(EXTRINSIC_RVIZ_WORLD_VEC.size() != 16)
    {
        ROS_ERROR("Wrong extrinsic matrix. size must be 16");
        exit(1);
    }
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXTRINSIC_RVIZ_WORLD(i, j) = EXTRINSIC_RVIZ_WORLD_VEC[i * 4 + j]; // i: row, j: col
        }
    }
    EXTRINSIC_RVIZ_WORLD_VEC.clear();

    // camera intrinsic
    int nTmpIdx = 0;
    for (int i = 0; i < INTRINSIC_CAM_MAT.total(); i++) {
        INTRINSIC_CAM_MAT.at<double>(i / INTRINSIC_CAM_MAT.cols, i % INTRINSIC_CAM_MAT.cols) = INTRINSIC_CAM_VEC[nTmpIdx++];
    }
    nTmpIdx = 0;
    for (int i = 0; i < CAM_DISTORTION_COEF_MAT.total(); i++) {
        CAM_DISTORTION_COEF_MAT.at<double>(i % CAM_DISTORTION_COEF_MAT.rows, i / CAM_DISTORTION_COEF_MAT.rows) = CAM_DISTORTION_COEF_VEC[nTmpIdx++];
    }
    nTmpIdx = 0;
    for (int i = 0; i < EXTRINSIC_BODY_CAM_MAT.total(); i++) {
        EXTRINSIC_BODY_CAM_MAT.at<double>(i / EXTRINSIC_BODY_CAM_MAT.cols, i % EXTRINSIC_BODY_CAM_MAT.cols) = EXTRINSIC_BODY_CAM_VEC[nTmpIdx++];
    }
    INTRINSIC_CAM_VEC.clear();
    CAM_DISTORTION_COEF_VEC.clear();
    std::cout << "\033[1;33mCamera Intrinsic\033[0m\n";
    std::cout << INTRINSIC_CAM_MAT << "\n";
    std::cout << "\033[1;33mCamera Distortion Coeff\033[0m\n";
    std::cout << CAM_DISTORTION_COEF_MAT << "\n";
    std::cout << "\033[1;33mExtrinsic T_{body}_{camera}\033[0m\n";
    std::cout << EXTRINSIC_BODY_CAM_MAT << "\n";

    cv::Mat T_c_b = EXTRINSIC_BODY_CAM_MAT.inv();
    _mT_c_b = T_c_b.clone();


    // Eigen matrix to tf
    _mTransformStamped.header.frame_id = "rviz";
    _mTransformStamped.child_frame_id = "world";
    eigenToTf(EXTRINSIC_RVIZ_WORLD, _mTransformStamped);
    sendRvizTF();

    std::cout << "\033[1;33mExtrinsic T_rviz_world: \n\033[0m";
    std::cout << _mTransformStamped.transform << "\n";

    if(SONAR_ANGLE_MIN>360 || SONAR_ANGLE_MIN < -360)
    {
        ROS_ERROR("sonar_angle_min param is out of range. must be -360 ~ 360");
        exit(1);
    }
    if(SONAR_ANGLE_MAX>360 || SONAR_ANGLE_MAX < -360)
    {
        ROS_ERROR("sonar_angle_max param is out of range. must be -360 ~ 360");
        exit(1);
    }
    if(SONAR_ANGLE_MIN > SONAR_ANGLE_MAX)
    {
        ROS_ERROR("sonar_angle param is wrong. must be sonar_angle_min <= sonar_angle_max");
        exit(1);
    }
    std::cout << "\033[1;33mSonar Range: \033[0m" << SONAR_ANGLE_MIN << " ~ " << SONAR_ANGLE_MAX << "\n";

    std::cout << "\n";
}
void MapDrawer::sendRvizTF(void)
{
    _mTransformStamped.header.stamp = ros::Time::now();
    _mBr.sendTransform(_mTransformStamped);
}
void MapDrawer::sendRvizTF(double Time)
{
    _mTransformStamped.header.stamp = ros::Time(Time);
    _mBr.sendTransform(_mTransformStamped);
}


/* @@@@@@@@@@@@@@@@@@@
@@@@@@@ Thread @@@@@@@
@@@@@@@@@@@@@@@@@@@ */
// Data processing
void MapDrawer::processData(void)
{
    while(1)
    {
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
        
        AssociatedSonarOdomData Data;
        if(!matchData(Data))continue;

        AssociatedSonarOdomData InterpolatedData = interpolateOdom(Data);
        if(!refineData(InterpolatedData))
        {
            continue;
        }
        addRawMapPoint(InterpolatedData);
    }
    
}
// Build map
void MapDrawer::buildMap(void)
{
    while(1)
    {
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        if(!syncImg())continue;

        Frame* ptrCurrFrame = nullptr;

        if(accumRawMapPoints(ptrCurrFrame))
        {
            renderMapPoints(ptrCurrFrame);
        }
        
        if(ptrCurrFrame != nullptr)
        {
            if(VIS_IMG)ptrCurrFrame->visImage();
            ptrCurrFrame->release();
            delete ptrCurrFrame;
        }

    }
    
}
// For debugging
void MapDrawer::debugging(void)
{
    if(!LOGGING)return;
    int nSeq = 1;
    while(1)
    {
        std::chrono::milliseconds dura(1000); // 1sec
        std::this_thread::sleep_for(dura);

        std::cout << "=======================================\n";
        std::cout << "\033[1;33m[Log: " << nSeq++ << "]\033[0m\n";
        std::cout << "Scan : " << _mnSonarScanNum << "\n";
        std::cout << "Raw Map Points Size: " << _mptrSonarMap->getRawSize() << "\n";
        std::cout << "Colored Map Points Size: " << _mptrSonarMap->getColoredSize() << "\n";
    }
}

/* @@@@@@@@@@@@@@@@@@@@@
@@@@@ ROS Callback @@@@@
@@@@@@@@@@@@@@@@@@@@@ */
void MapDrawer::callbackSonar(const sensor_msgs::LaserScan &msgSonar)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackSonar);
    
    if(msgSonar.header.stamp.toSec() != _mdPrevSonarCallbackStamp)_mqSonarBuf.push(msgSonar);
    // else std::cout << "Skip Sonar Callback\n";

    _mdPrevSonarCallbackStamp = msgSonar.header.stamp.toSec();

    // if(_mqSonarBuf.size() > MAX_QUEUE_SIZE)
    // {
    //     std::cout << "\033[1;33mSonar Data Buffer is Full!! Drop Old data\033[0m\n";
    //     _mqSonarBuf.pop();
    //     _mnDropSonarData++;
    // }
}
void MapDrawer::callbackOdom(const nav_msgs::Odometry &msgOdom)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackOdom);
    std::unique_lock<std::mutex> lock2(_mtxCallbackOdom2);

    if(msgOdom.header.stamp.toSec() != _mdPrevOdomCallbackStamp)
    {
        nav_msgs::Odometry tmpOdom = msgOdom;
        _mqOdomBuf.push(msgOdom);
        _mqOdomBuf2.push(tmpOdom);
    }
    // else std::cout << "Skip Odom Callback\n";

    _mdPrevOdomCallbackStamp = msgOdom.header.stamp.toSec();

    // if(_mqOdomBuf.size() > MAX_QUEUE_SIZE)
    // {
    //     std::cout << "\033[1;33mOdom Data Buffer is Full!! Drop Old data\033[0m\n";
    //     _mqOdomBuf.pop();
    //     _mnDropOdomData++;
    // }
}
void MapDrawer::callbackImg(const sensor_msgs::ImageConstPtr &msgImg)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackImg);

    if(msgImg->header.stamp.toSec() != _mdPrevImgCallbackStamp)_mqImgBuf.push(*msgImg);
    // else std::cout << "Skip Image Callback\n";

    _mdPrevImgCallbackStamp = msgImg->header.stamp.toSec();

    // if(_mqImgBuf.size() > MAX_QUEUE_SIZE)
    // {
    //     std::cout << "\033[1;33mImage Data Buffer is Full!! Drop Old data\033[0m\n";
    //     _mqImgBuf.pop();
    //     _mnDropImgData++;
    // }
}

/* @@@@@@@@@@@@@@@@@@@@@
@@@@@  For Thread  @@@@@
@@@@@@@@@@@@@@@@@@@@@ */
// Process Data
bool MapDrawer::matchData(AssociatedSonarOdomData &Data)
{
    if(_mqSonarBuf.empty() || _mqOdomBuf.empty())return false;

    // get sonar data
    sensor_msgs::LaserScan SonarData;
    {
        std::unique_lock<std::mutex> lock_sonar(_mtxCallbackSonar);
        SonarData = _mqSonarBuf.front();
        if(SonarData.header.stamp.toSec() < _mdPrevOdomStamp)
        {
            _mqSonarBuf.pop();
            return false;
        }
    }
    // get odom data
    nav_msgs::Odometry CurrOdomData;
    {
        std::unique_lock<std::mutex> lock_odom(_mtxCallbackOdom);
        CurrOdomData = _mqOdomBuf.front();
        if(CurrOdomData.header.stamp.toSec() < SonarData.header.stamp.toSec())
        {
            _mPrevOdom = CurrOdomData;
            _mdPrevOdomStamp = CurrOdomData.header.stamp.toSec();
            _mqOdomBuf.pop();
            return false;
        }
    }

    // make associated data
    Data = AssociatedSonarOdomData(_mPrevOdom, SonarData, CurrOdomData);

    // pop
    {
        std::unique_lock<std::mutex> lock_sonar(_mtxCallbackSonar);
        _mqSonarBuf.pop();
    }
    if(_mdPrevOdomStamp == 0)return false; // first data
    return true;
}
AssociatedSonarOdomData MapDrawer::interpolateOdom(AssociatedSonarOdomData Data)
{
    nav_msgs::Odometry prevOdom = Data.prevOdom;
    nav_msgs::Odometry currOdom = Data.currOdom;
    sensor_msgs::LaserScan SonarData = Data.SonarData;

    double dPrevTime = prevOdom.header.stamp.toSec();
    double dCurrTime = currOdom.header.stamp.toSec();
    double dSonarTime = SonarData.header.stamp.toSec();

    // interpolation ratio
    if (dSonarTime < dPrevTime || dSonarTime > dCurrTime)
    {
        ROS_WARN("SonarTime is out of the range between prevOdom and currOdom");
        _mnInterpolateErrorNum++;
        AssociatedSonarOdomData NULL_DATA;
        return NULL_DATA;
    }
    double dRatio = (dSonarTime - dPrevTime) / (dCurrTime - dPrevTime);

    // data interpolation
    nav_msgs::Odometry interpolatedOdom;
    interpolatedOdom.header.frame_id = "world";
    interpolatedOdom.header.stamp = SonarData.header.stamp;
        // position interpolation
    interpolatedOdom.pose.pose.position.x = lerp(prevOdom.pose.pose.position.x, currOdom.pose.pose.position.x, dRatio);
    interpolatedOdom.pose.pose.position.y = lerp(prevOdom.pose.pose.position.y, currOdom.pose.pose.position.y, dRatio);
    interpolatedOdom.pose.pose.position.z = lerp(prevOdom.pose.pose.position.z, currOdom.pose.pose.position.z, dRatio);
        // orientation interpolation
    tf::Quaternion prevQuat, currQuat, interpolatedQuat;
    tf::quaternionMsgToTF(prevOdom.pose.pose.orientation, prevQuat);
    tf::quaternionMsgToTF(currOdom.pose.pose.orientation, currQuat);
    interpolatedQuat = prevQuat.slerp(currQuat, dRatio);
    tf::quaternionTFToMsg(interpolatedQuat, interpolatedOdom.pose.pose.orientation);

    // push data
    AssociatedSonarOdomData interpolatedData;
    interpolatedData.currOdom = interpolatedOdom;
    interpolatedData.SonarData = SonarData;
    
    // publish interpolated odom
    sendRvizTF(interpolatedOdom.header.stamp.toSec());
    if(PUBLISH)_mPubInterOdom.publish(interpolatedOdom);

    return interpolatedData;
}

// Build Map
bool MapDrawer::refineData(AssociatedSonarOdomData &Data)
{
    sensor_msgs::LaserScan SonarData = Data.SonarData;
    if(SonarData.angle_max != SonarData.angle_min)return false;

    float fAngleDeg = SonarData.angle_max*(180.0 / M_PI);
    float fAngleDegN = fAngleDeg - 360;
    if(!((SONAR_ANGLE_MIN <= fAngleDeg && fAngleDeg <= SONAR_ANGLE_MAX) || 
        (SONAR_ANGLE_MIN <= fAngleDegN && fAngleDegN <= SONAR_ANGLE_MAX)))return false;

    sensor_msgs::LaserScan RefinedSonarData = SonarData;
    if(RefinedSonarData.ranges.size() != RefinedSonarData.intensities.size())
    {
        ROS_WARN("Sonar data has different size of ranges and intensities");
    }
    int nRageSize = RefinedSonarData.ranges.size();
    RefinedSonarData.ranges.clear();
    RefinedSonarData.intensities.clear();
    RefinedSonarData.ranges.reserve(nRageSize);
    RefinedSonarData.intensities.reserve(nRageSize);

    double dMaxIdx = 0;
    for(int i = 0; i < SonarData.intensities.size(); i++)
    {
        if(SonarData.intensities[i] <= MIN_INTENSITY_THD || SonarData.ranges[i] < SonarData.range_min || 
            SonarData.ranges[i] > SonarData.range_max)continue; 
        dMaxIdx = SonarData.intensities[i] > SonarData.intensities[dMaxIdx] ? i : dMaxIdx;
    }
    if(SonarData.ranges[dMaxIdx] == 0)return false;

    RefinedSonarData.ranges.push_back(SonarData.ranges[dMaxIdx]);
    RefinedSonarData.intensities.push_back(SonarData.intensities[dMaxIdx]);

    if(RefinedSonarData.ranges.size() == 0)return false;

    Data.SonarData = RefinedSonarData;

    _mnSonarScanNum++;
    return true;
}
void MapDrawer::addRawMapPoint(AssociatedSonarOdomData Data)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr SonarCloud_sonar(new pcl::PointCloud<pcl::PointXYZI>());
    laserScanToPointCloud(Data.SonarData, SonarCloud_sonar);
    pcl::PointCloud<pcl::PointXYZI>::Ptr SonarCloud_body = transformPoint(SonarCloud_sonar, _mT_b_s);

    auto OdomMatrix = odomToTransformationMatrix(Data.currOdom);
    pcl::PointCloud<pcl::PointXYZI>::Ptr SonarCloud_world = transformPoint(SonarCloud_body, OdomMatrix);
    
    _mptrSonarMap->addMapPoints(SonarCloud_world);
    if(SonarCloud_world->points.size() > 1)
    {
        // Something Wrong in refineData
    }

    float x = SonarCloud_world->points[0].x;
    float y = SonarCloud_world->points[0].y;
    float z = SonarCloud_world->points[0].z;
    // std::cout << x << " " << y << " " << z << "\n";

    MapPoint* ptrRawMapPoint = new MapPoint();
    ptrRawMapPoint->setTimeStamp(Data.SonarData.header.stamp.toSec());
    ptrRawMapPoint->setPosition(x,y,z);

    {
        std::unique_lock<std::mutex> lock(_mtxRawMapPoint);
        _mqRawMapPointBuf.push(ptrRawMapPoint);
    }

    if(PUBLISH)
    {
        sendRvizTF(Data.SonarData.header.stamp.toSec());
        _mPubRawMap.publish(_mptrSonarMap->getAllRawCloud());
    }

    SonarCloud_body->clear();
    SonarCloud_sonar->clear();
}
bool MapDrawer::syncImg(void)
{
    if(_mqOdomBuf2.empty())return false;
    if(_mqImgBuf.empty())return false;

    // get image data
    sensor_msgs::Image CurrImg;
    {
        std::unique_lock<std::mutex> lock_img(_mtxCallbackImg);
        CurrImg = _mqImgBuf.front();
        if(CurrImg.header.stamp.toSec() < _mdPrevOdomStamp2)
        {
            _mqImgBuf.pop();
            return false;
        }
    }

    // get odom data
    nav_msgs::Odometry CurrOdomData;
    {
        std::unique_lock<std::mutex> lock_odom(_mtxCallbackOdom2);
        CurrOdomData = _mqOdomBuf2.front();
        if(CurrOdomData.header.stamp.toSec() < CurrImg.header.stamp.toSec())
        {
            _mPrevOdom2 = CurrOdomData;
            _mdPrevOdomStamp2 = CurrOdomData.header.stamp.toSec();
            _mqOdomBuf2.pop();
            return false;
        }
    }

    // pop
    {
        std::unique_lock<std::mutex> lock_img(_mtxCallbackImg);
        _mqImgBuf.pop();
    }

    if(_mdPrevOdomStamp2 == 0)return false; // first data
    
    // interpolation //
    double dPrevTime = _mPrevOdom2.header.stamp.toSec();
    double dCurrTime = CurrOdomData.header.stamp.toSec();
    double dImgTime = CurrImg.header.stamp.toSec();
    
    // interpolation ratio
    if (dImgTime < dPrevTime || dImgTime > dCurrTime)
    {
        ROS_WARN("SonarTime is out of the range between prevOdom and currOdom");
        return false;
    }
    double dRatio = (dImgTime - dPrevTime) / (dCurrTime - dPrevTime);

    // data interpolation
    nav_msgs::Odometry interpolatedOdom;
    interpolatedOdom.header.frame_id = "world";
    interpolatedOdom.header.stamp = CurrImg.header.stamp;
        // position interpolation
    interpolatedOdom.pose.pose.position.x = lerp(_mPrevOdom2.pose.pose.position.x, CurrOdomData.pose.pose.position.x, dRatio);
    interpolatedOdom.pose.pose.position.y = lerp(_mPrevOdom2.pose.pose.position.y, CurrOdomData.pose.pose.position.y, dRatio);
    interpolatedOdom.pose.pose.position.z = lerp(_mPrevOdom2.pose.pose.position.z, CurrOdomData.pose.pose.position.z, dRatio);
        // orientation interpolation
    tf::Quaternion prevQuat, currQuat, interpolatedQuat;
    tf::quaternionMsgToTF(_mPrevOdom2.pose.pose.orientation, prevQuat);
    tf::quaternionMsgToTF(CurrOdomData.pose.pose.orientation, currQuat);
    interpolatedQuat = prevQuat.slerp(currQuat, dRatio);
    tf::quaternionTFToMsg(interpolatedQuat, interpolatedOdom.pose.pose.orientation);
    // frame
    cv::Mat icCurrImg = convertROSImage2CvMat(CurrImg);
    Frame* ptrCurrFrame = new Frame();
    ptrCurrFrame->setOdom(interpolatedOdom);
    ptrCurrFrame->setRawImg(icCurrImg);

    // push
    {
        std::unique_lock<std::mutex> lock_frame(_mtxSyncFrame);
        _mqSyncFrameBuf.push(ptrCurrFrame);
    }

    // std::cout << interpolatedOdom.pose.pose.position.x << " "
    //           << interpolatedOdom.pose.pose.position.y << " "
    //           << interpolatedOdom.pose.pose.position.z << "\n";

    return true;
}

bool MapDrawer::accumRawMapPoints(Frame* &ptrCurrFrame)
{
    if(_mqSyncFrameBuf.empty())return false;
    if(_mqRawMapPointBuf.empty())return false;

    {
        std::unique_lock<std::mutex> lock_frame(_mtxSyncFrame);
        ptrCurrFrame = _mqSyncFrameBuf.front();
    }

    while (!_mqRawMapPointBuf.empty())
    {
        std::unique_lock<std::mutex> lock_point(_mtxRawMapPoint);
        MapPoint* ptrRawMapPoint = _mqRawMapPointBuf.front();

        if(ptrRawMapPoint->getTimeStamp() > ptrCurrFrame->getTimeStamp())break;

        ptrCurrFrame->pushMapPoint(ptrRawMapPoint);
        _mqRawMapPointBuf.pop();
    }

    {
        std::unique_lock<std::mutex> lock_frame(_mtxSyncFrame);
        _mqSyncFrameBuf.pop();
    }

    return true;
}

void MapDrawer::renderMapPoints(Frame* &ptrCurrFrame)
{
    if(ptrCurrFrame->getMapPointsSize() == 0)return;

    std::vector<cv::Point3f> vpMapPoints;
    std::vector<cv::Point2f> vpProjectedPts;
    projectMapPoints(ptrCurrFrame, vpMapPoints, vpProjectedPts);

    cv::Mat visImg = ptrCurrFrame->getProcessedImg();
    cv::Mat overlayImg = visImg.clone();

    std::vector<int> vnValidIdx;
    int imgWidth = visImg.cols;
    int imgHeight = visImg.rows;
    for (int i = 0; i < vpProjectedPts.size(); i++) {
        int x = static_cast<int>(std::round(vpProjectedPts[i].x));
        int y = static_cast<int>(std::round(vpProjectedPts[i].y));
        if (x >= 0 && x < imgWidth && y >= 0 && y < imgHeight) {      
            cv::circle(visImg, vpProjectedPts[i], 5, cv::Scalar(0,0,255), -1);

            vnValidIdx.push_back(i);
        }
    }
    // cv::imshow("projected", visImg);
    // cv::waitKey(1);
    ptrCurrFrame->setProjectedImg(visImg);

    // colorize
    if(vnValidIdx.size() == 0)return;


    cv::Mat icProcessedImg = ptrCurrFrame->getProcessedImg();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrColoredPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i = 0; i < vnValidIdx.size(); i++)
    {
        int nIdx = vnValidIdx[i];
        pcl::PointXYZRGB tmpColoredPoint;
        
        // get clolor
        int x = static_cast<int>(std::round(vpProjectedPts[nIdx].x));
        int y = static_cast<int>(std::round(vpProjectedPts[nIdx].y));
        auto color = icProcessedImg.at<cv::Vec3b>(y,x);

        uint8_t blue = static_cast<uint8_t>(color[0]);
        uint8_t green = static_cast<uint8_t>(color[1]);
        uint8_t red = static_cast<uint8_t>(color[2]);

        // //
        tmpColoredPoint.x = vpMapPoints[i].x;
        tmpColoredPoint.y = vpMapPoints[i].y;
        tmpColoredPoint.z = vpMapPoints[i].z;

        tmpColoredPoint.r = red;
        tmpColoredPoint.g = green;
        tmpColoredPoint.b = blue;

        //
        ptrColoredPoints->points.push_back(tmpColoredPoint);
    }
    ptrColoredPoints->width = ptrColoredPoints->points.size();
    ptrColoredPoints->height = 1;
    ptrColoredPoints->is_dense = true;

    _mptrSonarMap->addColoredMapPoints(ptrColoredPoints);
    
    if(PUBLISH)
    {
        _mPubColoredMap.publish(_mptrSonarMap->getAllColoredCloud());
    }
}

void MapDrawer::projectMapPoints(Frame* &ptrFrame, std::vector<cv::Point3f> &MapPoints, std::vector<cv::Point2f> &ProjectedPoints)
{
    MapPoints.clear();
    ProjectedPoints.clear();

    cv::Mat T_w_b = ptrFrame->get_T_w_b();
    cv::Mat T_c_w = _mT_c_b*(T_w_b.inv());

    cv::Mat TransVec;
    cv::Mat RotVec;
    convertTransformMat2Vec(T_c_w, TransVec, RotVec);

    MapPoints = ptrFrame->getMapPointsPos();

    cv::projectPoints(MapPoints, RotVec, TransVec, INTRINSIC_CAM_MAT, cv::Mat::zeros(5,1, CV_64F), ProjectedPoints);
}

void MapDrawer::saveMap(void)
{
    if(!SAVE_MAP)return;
    boost::filesystem::path dir(std::string(ROOT_DIR) + "map");
    boost::filesystem::create_directory(dir);
    std::string sFilePath = std::string(ROOT_DIR) + "map/mapping.pcd";

    _mptrSonarMap->saveMap(sFilePath);

    std::cout << "\033[1;33mMap saved to: " << sFilePath << "\033[0m\n";
}