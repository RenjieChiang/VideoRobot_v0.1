//
// Created by tianbot on 2021/3/6.
//

#include "MyRobot.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_robot_test");
    ros::NodeHandle nh;
    MyRobot chiangRobot(nh);
    chiangRobot.motionEnable();
//    chiangRobot.moveLinebTest();
    chiangRobot.readCameraData();
    return 0;
}