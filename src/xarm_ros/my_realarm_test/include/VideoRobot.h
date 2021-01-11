#ifndef _VIDEOROBOT_H_
#define _VIDEOROBOT_H_

#include "fstream"
#include "string"
#include "iostream"
#include "xarm_driver.h"
#include "ros/ros.h"

    typedef struct my_home_position
    {
        /* data */
      typedef struct position
      {
          /* data */
          float x;
          float y;
          float z;
      }position;

      typedef struct rotation
      {
          /* data */
          float r;//绕x rad
          float p;//绕y
          float y;//绕z
      }rotation;
    }my_home;

class VideoRobot
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient motion_ctrl_client_ ;
	ros::ServiceClient set_mode_client_ ;
	ros::ServiceClient set_state_client_ ;
  	ros::ServiceClient go_home_client_ ;
	ros::ServiceClient servo_cart_client_ ;
    ros::ServiceClient move_lineb_client_;
    ros::Publisher sleep_pub_;

    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::Move move_srv_;
    //自定义的home位置
    my_home my_home0;
    //机械臂状态值    
    int mode;
    int state;
    /* 
      Mode 0 : 基于xArm controller规划的位置模式；move_jiont/line/lineb/tool
    Mode 1 : 基于外部轨迹规划器的位置模式；move_servo_cart/servoj
    Mode 2 : 自由拖动(零重力)模式。
    state 0  moveit实现的运动、servo需要0
    state 1 
    */
    std::string filename;
    std::vector<std::vector<float>> cam_posedata;//读取文件获得的数据点
    std::vector<std::vector<float>> rob_posedata;//插值后的pose
    float startpose[6];//起始位置，所有robpose会加上这个值

public:
    //constuctor
    VideoRobot::VideoRobot();
    VideoRobot::VideoRobot(ros::NodeHandle &nh_);
    //destrucor
    VideoRobot::~VideoRobot();
    void setHome(float x_ , float y_ , float z_ );
    void setHome(float x_ , float y_ , float z_ , float rotation_r , float rotation_p , float rotation_y );
    void setMode(int mode_);
    void setState(int state_);
    void setStartpose(float startpose_[6]);
    int motionEnable();
    int moveLineb(std::vector<float> pose_);
    int readCameraData();//默认文件存储位置
    int readCameraData(string filename_);
    int interDataToPose(int n_);//@param n_为插值点数
    int goMyHome();
    int goVideo();
};
#endif