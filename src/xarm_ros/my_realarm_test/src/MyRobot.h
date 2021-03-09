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

#define DefualtFileName "/home/tianbot/testcppcode/transdataTRUE"//默认文件存储位置

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
    std_msgs::Float32 sleep_msg;

    MyHome myHome_0;//自定义home位置，todo 目前未通过自带home api操作
    //     机械臂状态
    //   Mode 0 : 基于xArm controller规划的位置模式；是控制器启动后默认进入的模式
    //   Mode 1 : 基于外部轨迹规划器的位置模式；此模式下,机械臂可以接受以 100Hz 频率发送的关节位置指令
    //   Mode 2 : 自由拖动(零重力)模式。
    //   STATE状态 0:开启运动。
    //   可以理解为准备好运动或运动就绪状态。在此状态下,机械臂能够正常响应和执行
    //   运动指令。 如果机械臂从错误、断电或停止状态(状态 4)中恢复,请记得在继续
    //    发送运动指令之前设置状态为 0,否则发送的指令会被丢弃。
    //    ● 状态 3:暂停状态。
    //    暂停当前在执行的运动,并可以通过再次设置状态 0 在中断处继续恢复运行。
    //    状态 4:停止状态。
    //    终止当前的运动,并清空已缓存的后续指令。需要设置状态 0 之后方可继续运动
    enum {Controller = 0, ExternalPlanner = 1, FreeGravity = 2} mode;
    enum {MoveEnable = 0, Pause = 3} state;//todo 可能不正确
    //读取相机数据
    std::string filename_;
    std::vector<std::vector<float>> cam_pose;//读取文件得到的数据
    std::vector<std::vector<float>> rob_pose;//插值后数据
    float start_pose[6];//起始位置，所以rob_pose会附加上这个值
    static double LinearModelPoly9(const double *p_, double x);
    static double GeneralModelSin8(const double ** p_, double x);

public:
//    MyRobot();
    ~MyRobot();
    explicit MyRobot(ros::NodeHandle & nh);
    void setHome(float x_ , float y_ , float z_ );
    void setHome(float x_ , float y_ , float z_ , float rotation_r , float rotation_p , float rotation_y );
    void setMode(int mode_);
    void setState(int state_);
    void setStartpose(const float startpose_[6]);
    void motionEnable();
    void moveLinebTest();
    void moveLineb(const std::vector<float> & pose_);
    void readCameraData(const std::string & filename = DefualtFileName);//默认文件存储位置
    void interDataToPose(int n_);//@param n_为插值点数
    void goMyHome();
    void moveServoCart(const std::vector<std::vector<float>> & servo_pose);
    void goVideo();
    void calculate();
};


#endif //CHIANG_XARM_WS_MYROBOT_H
