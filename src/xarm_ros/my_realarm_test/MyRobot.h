//
// Created by tianbot on 2021/3/3.
// MyRobot ----> video
//

#ifndef CHIANG_XARM_WS_MYROBOT_H
#define CHIANG_XARM_WS_MYROBOT_H

#include "fstream"
#include "string"
#include "iostream"
#include "xarm_driver.h"
#include "ros/ros.h"

typedef struct position3D
{
    float x;
    float y;
    float z;
}position;

typedef struct rotation
{
    float r;//绕x rad
    float p;//绕y
    float y;//绕z
}rotation;

typedef struct MyHomePosition
{
    position home_position;
    rotation home_rotation;
}MyHome;

class MyRobot
{
private:
    //xarm + ros 机械臂基础组件
    ros::NodeHandle nh_;
    ros::ServiceClient motion_ctrl_client_ ;
    ros::ServiceClient set_mode_client_ ;//状态和模式设置
    ros::ServiceClient set_state_client_ ;
    ros::ServiceClient go_home_client_ ;//自带api的归位服务
    ros::ServiceClient servo_cart_client_ ;//伺服运动api
    ros::ServiceClient move_lineb_client_;//圆弧优化拐点的api
    ros::Publisher sleep_pub_;
    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::Move move_srv_;

    MyHome myHome_0;//自定义home位置，todo 目前未通过自带home api操作
    //     机械臂状态
    //   Mode 0 : 基于xArm controller规划的位置模式；是控制器启动后默认进入的模式
    //   Mode 1 : 基于外部轨迹规划器的位置模式；此模式下,机械臂可以接受以 100Hz 频率发送的关节位置指令
    //   Mode 2 : 自由拖动(零重力)模式。
    enum mode {Controller = 0, ExternalPlanner = 1, FreeGravity = 2};
    enum state {MoveEnable = 0, Pause = 1};//todo 可能不正确

    //读取相机数据
    std::string filename_;
    std::vector<>
};


#endif //CHIANG_XARM_WS_MYROBOT_H
