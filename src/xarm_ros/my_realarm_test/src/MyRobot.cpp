//
// Created by tianbot on 2021/3/3.
//

#include "MyRobot.h"

//const std::string default_filename = "/home/tianbot/testcppcode/transdataTRUE" ;
//MyRobot::MyRobot(){}

MyRobot::MyRobot(ros::NodeHandle &nh)
{
    nh_ = nh;
    motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
    set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
    set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
    go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
    servo_cart_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart");
    move_lineb_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_lineb");
    sleep_pub_ = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);
    mode = Controller;
    state = MoveEnable;
    float pose[6] = {350, 0, 250, -3.14, 0, -3.14/2};
    for (int i = 0; i < 6; ++i) {
        start_pose[i] = pose[i];
    }
    myHome_0.home_position.x = start_pose[0];
    myHome_0.home_position.y = start_pose[1];
    myHome_0.home_position.z = start_pose[2];
    myHome_0.home_rotation.r = start_pose[3];
    myHome_0.home_rotation.p = start_pose[4];
    myHome_0.home_rotation.y = start_pose[5];
}

MyRobot::~MyRobot()
{
    //恢复到正常模式
    mode = Controller;
    state = MoveEnable;
}

void MyRobot::setHome(float x_, float y_, float z_)
{
    myHome_0.home_position.x = x_;
    myHome_0.home_position.y = y_;
    myHome_0.home_position.z = z_;
}

void MyRobot::setHome(float x_, float y_, float z_, float rotation_r, float rotation_p, float rotation_y)
{
    myHome_0.home_position.x = x_;
    myHome_0.home_position.y = y_;
    myHome_0.home_position.z = z_;
    myHome_0.home_rotation.r = rotation_r;
    myHome_0.home_rotation.p = rotation_p;
    myHome_0.home_rotation.y = rotation_y;
}
//设置mode，抛出exception 输入错误和call错误
void MyRobot::setMode(int mode_)
{
    if (mode_ == 0) mode = Controller;
    else if (mode_ == 1) mode = ExternalPlanner;
    else if (mode_ == 2) mode = FreeGravity;
    else throw std::invalid_argument("state should be Controller = 0, ExternalPlanner = 1, FreeGravity = 2");
    set_int16_srv_.request.data = static_cast<int>(mode);
    if (set_mode_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
    } else throw std::runtime_error("Failed to call service set_mode");
}
//设置state，抛出exception 输入错误和call错误
void MyRobot::setState(int state_)
{
    if (state_ == 0) state = MoveEnable;
    else if (state_ == 3) state = Pause;
    else throw std::invalid_argument("state should be MoveEnable = 0, Pause = 3");
    set_int16_srv_.request.data = static_cast<int>(state);
    if (set_state_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
    } else throw std::runtime_error("Failed to call service set_state");
}

void MyRobot::setStartpose(const float *startpose_)
{
    for (int i = 0; i < 6; ++i)
    {
        start_pose[i] = startpose_[i];
    }
}

void MyRobot::motionEnable()
{
    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;
    if(motion_ctrl_client_.call(set_axis_srv_))
    {
        ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
    } else throw std::runtime_error("Failed to call service motion_ctrl");
}
//异常：运动服务call失败
void MyRobot::moveLinebTest() //未使用pose目前为测试
{
    move_srv_ .request.mvvelo = 20.0 / 57.0;
    move_srv_ .request.mvacc = 1000;
    move_srv_ .request.mvtime = 0;
    move_srv_.request.pose={350,0,250,-3.14,0,-3.14/2};
    if(move_lineb_client_.call(move_srv_))
    {
        ROS_INFO("%s\n", move_srv_.response.message.c_str());
    }
    else throw std::runtime_error("Failed to call service move_lineb");
}
void MyRobot::moveLineb(const std::vector<float> & pose_)
{
    move_srv_.request.pose = pose_;
    move_srv_ .request.mvvelo = 20.0 / 57.0;
    move_srv_ .request.mvacc = 1000;
    move_srv_ .request.mvtime = 0;
    if(move_lineb_client_.call(move_srv_))
    {
        ROS_INFO("%s\n", move_srv_.response.message.c_str());
    }
    else throw std::runtime_error("Failed to call service move_lineb");
}

//异常： 文件打开失败
void MyRobot::readCameraData(const std::string & filename)
{
    filename_ = filename;
    std::vector<float> single_pose;//使用两维动态数组，一个用于存各个pose，一个将他们集合起来，
    std::ifstream myfile;
    std::cout<<"opening data: "<<filename<<"\n";
    myfile.open(filename,std::ios::in);
    if (!myfile.is_open())
        throw std::logic_error("Open file fail!");
    int count = 0;
    std::string temp;
    //判断数据行数
    while (std::getline(myfile,temp))
    {
        if(!temp.empty())
        {
            count++;
        }
    }
    std::cout<<"共有"<<count<<"行数据"<<std::endl;
    myfile.close();
    float num[6];
    myfile.open(filename,std::ios::in);
    for(int i = 0; i < count; i ++)
    {
        for(float & j : num)
        {
            myfile>>j;
        }
        single_pose.push_back(10*num[0]+350);
        single_pose.push_back(10*num[1]);
        single_pose.push_back(10*num[2]+250);
        single_pose.push_back(-3.14);
        single_pose.push_back(0);
        single_pose.push_back(num[5]/180*3.14-3.14/2);
        //数据顺序为
        // 0 1 2  3   4   5
        // x y z r_r r_p r_y
        cam_pose.push_back(single_pose);
        single_pose.clear();
    }
}

void MyRobot::interDataToPose(int n_) //todo 数据插值与处理
{
    rob_pose = cam_pose;
}
//异常：使能失败或movelineb失败导致异常
void MyRobot::goMyHome()
{
    std::vector<float> home_vec;
    home_vec.push_back(myHome_0.home_position.x);
    home_vec.push_back(myHome_0.home_position.y);
    home_vec.push_back(myHome_0.home_position.z);
    home_vec.push_back(myHome_0.home_rotation.r);
    home_vec.push_back(myHome_0.home_rotation.p);
    home_vec.push_back(myHome_0.home_rotation.y);
    try
    {
        motionEnable();
        moveLineb(home_vec);
    }
    catch (std::exception & e)
    {
        std::cout << e.what() << std::endl;
        throw std::runtime_error("go home fail! ");
    }
}
//异常：servo运动失败
void MyRobot::moveServoCart(const std::vector<std::vector<float>> & servo_pose)
{
    move_srv_.request.mvvelo = 0;
    move_srv_.request.mvacc = 0;
    move_srv_.request.mvtime = 0;//0为普通模式，1为步进模式（坐标为相对值）
    for (const auto & i : servo_pose)
    {
        move_srv_.request.pose = i;
        if (servo_cart_client_.call(move_srv_))
        {
            ROS_INFO("%s\n", move_srv_.response.message.c_str());
        } else throw std::runtime_error("servo move fail!!!!");
        usleep(41666/2); //48HZ
    }
}

void MyRobot::goVideo()
{
    try {
        moveServoCart(rob_pose);
    }
    catch (std::exception & e)
    {
        std::cout << e.what() << std::endl;
        throw std::runtime_error("go video fail!");
    }
}

