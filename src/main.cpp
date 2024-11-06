/*
Author: Sanghyun Park
---
CoCEL(Computational Control Engineering Lab)
POSTECH(Pohang University of Science and Technology)
*/
#include "visual_sonar_rgb_mapping/MapDrawer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_rgb_mapping");
    ros::NodeHandle nh_("~");

    MapDrawer SonarMapper(nh_);

    std::thread thProcessData(&MapDrawer::processData, &SonarMapper);
    std::thread thBuildMap(&MapDrawer::buildMap, &SonarMapper);
    std::thread thDebugging(&MapDrawer::debugging, &SonarMapper);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    // ros::spin();

    signal(SIGINT, mySigintHandler);
    ros::waitForShutdown();

    if(!ros::ok())
    {
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
        SonarMapper.saveMap();
    }

    return 0;
}