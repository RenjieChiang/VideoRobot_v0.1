#include "ros/ros.h"
#include<xarm_driver.h>
#include<iostream>
#include<fstream>
#include<string>
#include<moveit/move_group_interface/move_group_interface.h>
const std::string filename="/home/tianbot/testcppcode/transdataTRUE" ;

int go_home_test(xarm_msgs::Move srv, ros::ServiceClient client)
{
    srv.request.mvvelo = 20.0 / 57.0;
    srv.request.mvacc = 1000;
    srv.request.mvtime = 0;
    if(client.call(srv))
    {
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service go home");
        return 1;
    }
    return 0;
}

int readcameradata(std::vector< std::vector<float>>& posedata)
{
    float num[200][10];
    std::vector<float> poseda_;
//使用两维动态数组，一个用于存各个pose，一个将他们集合起来，api接口数据结构为vector<float>
   std::ifstream myfile;
   std::cout<<"opening data: "<<filename<<"\n";
   myfile.open(filename,std::ios::in);
  if(!myfile.is_open())
  {
    std::cout<<"failed to open file!";
    return 1;
  }
  
std::string temp;
  int i,j,count=0;
  while (std::getline(myfile,temp))
    {
        if(temp.size()>0)
        {
            count++;
            std::cout<<temp<<std::endl;
        }
    }
    std::cout<<"共有"<<count<<"行数据"<<std::endl;
    myfile.close();
    //以上为判断数据行数

    std::ifstream myfile1;
    myfile1.open(filename);
     for(i=0;i<count;i++)
    {
        for(j=0;j<6;j++)
        {
            myfile1>>num[i][j];
        }
    }
    myfile1.close();
    // geometry_msgs::Pose target_pose;
    //数据暂时存储一下
    
    for (i = 0; i < count; i++)
    {
      poseda_.push_back(10*num[i][3]+350);
      poseda_.push_back(10*num[i][4]);
      poseda_.push_back(10*num[i][5]+250);
      poseda_.push_back(-3.14);
      poseda_.push_back(0);
      poseda_.push_back(num[i][1]/180*3.14-3.14/2);
      
      posedata.push_back(poseda_);
      poseda_.clear();
    //根据实际情况处理数据
    }
    return 0;
}

int move_lineb_test(xarm_msgs::Move srv, ros::ServiceClient client)
{


    std::vector< std::vector<float>>posedata;

    if(readcameradata(posedata)==1)
    {
        return 1;
    }
    ROS_INFO("1111111111111111");
    srv.request.mvvelo = 100;
    srv.request.mvacc = 1000;
    srv.request.mvtime = 0;
    srv.request.mvradii = 20;
    
    //回到设立的初始点
    srv.request.pose={350,0,250,-3.14,0,-3.14/2};
    if(client.call(srv))
    {
                ROS_INFO("%s\n", srv.response.message.c_str());
    }
        else
    {
                ROS_ERROR("Failed to call service move_lineb");
                return 1;
     }
     

    
//走点功能
    // int i;
    // for ( i = 0; i < posedata.size(); i++)
    // {
           
    //         srv.request.pose=posedata[i];
    //             ROS_INFO("1111111111111111");
    //         if(client.call(srv))
    //         {
    //             ROS_INFO("%s\n", srv.response.message.c_str());
    //         }
    //         else
    //         {
    //             ROS_ERROR("Failed to call service move_lineb");
    //             return 1;
    //         }
    // }
    return 0;
}



	// std::vector<float> pose[5] = {  {300, 0, 100, -3.14, 0, 0},
    //                                 {300, 100, 100, -3.14, 0, 0},
    //                                 {400, 100, 100, -3.14, 0, 0},
    //                                 {400, -100, 100, -3.14, 0, 0},
    //                                 {300, -100, 100, -3.14, 0, 0}};

    


int main(int argc,char **argv)
{
    ros::init(argc, argv, "realarm_test");
	ros::NodeHandle nh;
    
	
    nh.setParam("/xarm/wait_for_finish", true); // return after motion service finish
    ros::Publisher sleep_pub_ = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);
    ros::ServiceClient motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
	ros::ServiceClient set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
	ros::ServiceClient set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  	ros::ServiceClient go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
	ros::ServiceClient move_lineb_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_lineb");
	ros::ServiceClient move_servoj_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servoj");

    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::Move move_srv_;
    
    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;
    
    if(motion_ctrl_client_.call(set_axis_srv_))
    {
        ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service motion_ctrl");
        return 1;
    }

    set_int16_srv_.request.data = 0;
    if(set_mode_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service set_mode");
        return 1;
    }//检查模式  

    set_int16_srv_.request.data = 0;
    if(set_state_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service set_state");
        return 1;
    }//检查状态模式

    // if(go_home_test(move_srv_, go_home_client_) == 1) 
    //     return 1;
//调用homeclient回到home位置

   // MOVE_LINEB need some additional configurations: wait=False, sleep before sending commands
   nh.setParam("/xarm/wait_for_finish", false); // This configuration is CRITICAL for move_lineb!
   std_msgs::Float32 sleep_msg;
   sleep_msg.data = 1.0;
   sleep_pub_.publish(sleep_msg);
   if(move_lineb_test(move_srv_, move_lineb_client_) == 1)
        return 1;
   nh.setParam("/xarm/wait_for_finish", true);//重点设置wait !


}