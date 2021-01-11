#include "VideoRobot.h"

const std::string filename_default = "/home/tianbot/testcppcode/transdataTRUE"
const float startpose_default[6] = {350, 0, 250, -3.14, 0, -3.14/2};

VideoRobot::VideoRobot()
{

}

//前提：ros::init()
 VideoRobot::VideoRobot(ros::NodeHandle &nh_)
 {
    nh = nh_;
    nh.setParam("/xarm/wait_for_finish", true); // return after motion service finish
   /*
     运动服务调用在默认情况下会立刻返回，
    如果希望等待运动结束之后再返回, 需要提前设置 
    ros parameter "/xarm/wait_for_finish" 为 true.
    */
    motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
	set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
	set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  	go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
	servo_cart_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart");
    move_lineb_client_ = nh.serviceClient<xarm_msgs::Move> ("/xarm/move_lineb");
    sleep_pub_ = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);

    mode = 0;
    state = 0;
    filename = filename_default;
    my_home0.position.x = startpose_default[0];
    my_home0.position.y = startpose_default[1];
    my_home0.position.z = startpose_default[2];
    my_home0.rotation.r = startpose_default[3];
    my_home0.rotation.p = startpose_default[4];
    my_home0.rotation.y = startpose_default[5];

    for (size_t i = 0; i < 6; i++)
    {
        startpose[i] = startpose_default[i];
    }
    
    
 }

//destrucor
    VideoRobot::~VideoRobot()
    {

    }

    void VideoRobot::setHome(float x_ , float y_ , float z_ )
    {
        my_home0.position.x = x_;
        my_home0.position.y = y_;
        my_home0.position.z = z_;
    }

    void VideoRobot::setHome(float x_ , float y_ , float z_ , float rotation_r , float rotation_p , float rotation_y )
    {
        my_home0.position.x = x_;
        my_home0.position.y = y_;
        my_home0.position.z = z_;
        my_home0.rotation.r = rotation_r;
        my_home0.rotation.p = rotation_p;
        my_home0.rotation.y = rotation_y;
    }

    int VideoRobot::setMode(int mode_)
    {
        mode = mode_;
        set_init16_srv_.request.data = mode_;
        if (set_mode_client_.call(set_int16_srv_))
        {
            ROS_INFO("%s\n", set_int16_srv.response.message.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service set_mode");
            return 1;
        }
        return 0;
        
    }

    int VideoRobot::setState(int state_)
    {
        state = state_;
        set_int16_srv_.request.data = state_;
        if(set_state_client_.call(set_int16_srv_))
        {
            ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service set_state");
            return 1;
        }
        return 0;
    }

    void VideoRobot::setStartpose(float startpose_[6])
    {
        for (size_t i = 0; i < 6; i++)
        {
            startpose[i] = startpose_[i];
        }
    }
    
    int VideoRobot::readCameraData()
    {

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
            // std::cout<<temp<<std::endl;
        }
    }
     std::cout<<"共有"<<count<<"行数据"<<std::endl;
     myfile.close();
     //以上为判断数据行数

     std::ifstream myfile1;
     myfile1.open(filename);
     float tempfloat = 0.0;
     std::vector<float> tempvf;
     for(i=0;i<count;i++)
     {
         for(j=0;j<6;j++)
         {
             myfile1>>tempfloat;
             tempvf.push_back(tempfloat);
         }
         cam_posedata.push_back(tempvf);
         tempvf.clear();
     }
    myfile1.close();
    }

    int VideoRobot::readCameraData(string filename_)
    {
        filename = filename_;
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
            // std::cout<<temp<<std::endl;
        }
     }
     std::cout<<"共有"<<count<<"行数据"<<std::endl;
     myfile.close();
     //以上为判断数据行数

     std::ifstream myfile1;
     myfile1.open(filename);
     float tempfloat = 0.0;
     std::vector<float> tempvf;
     for(i=0;i<count;i++)
     {
         for(j=0;j<6;j++)
         {
             myfile1>>tempfloat;
             tempvf.push_back(tempfloat);
         }
         cam_posedata.push_back(tempvf);
         tempvf.clear();
     }
    myfile1.close();
    }

    int VideoRobot::interDataToPose(int n_)//@param n_为插值点数
    {
        std::vector<float> robtempvf;
        for (size_t i = 0; i < cam_posedata.size() - 1; i++)
        {
            float k[6];
            for (size_t c = 0; c < 6; c++)
            {
                k[c] = cam_posedata[i+1][c] - cam_posedata[i][c]; 
             }
            for (size_t j = 0; j < n_+1; j++)
            {
                robtempvf.push_back(10 * (cam_posedata[i][0] + k[0]*j/n_) + startpose[0]);
                robtempvf.push_back(10 * (cam_posedata[i][1] + k[1]*j/n_) + startpose[1]);
                robtempvf.push_back(10 * (cam_posedata[i][2] + k[2]*j/n_) + startpose[2]);
                robtempvf.push_back(startpose[3]);
                robtempvf.push_back(startpose[4]);
                robtempvf.push_back((cam_posedata[i][5] + k[5]*j/n_)/180*3.14 + startpose[5]);
                rob_posedata.push_back(robtempvf);
                robtempvf.clear();
            }
        }
        //最后一组数据
        robtempvf.push_back(10 * cam_posedata[i][0] + startpose[0]);
        robtempvf.push_back(10 * cam_posedata[i][1] + startpose[1]);
        robtempvf.push_back(10 * cam_posedata[i][2] + startpose[2]);
        robtempvf.push_back(startpose[3]);
        robtempvf.push_back(startpose[4]);
        robtempvf.push_back(cam_posedata[i][5]/180*3.14 + startpose[5]);
        rob_posedata.push_back(robtempvf);
        robtempvf.clear();
    }

    int VideoRobot::motionEnable()
    {
        set_axis_srv_.request.id = 8;
        set_axis_srv_.request.data = 1;

        if (motion_ctrl_client_.call(set_axis_srv_))
        {
            ROS_INFO("%s\n",set_axis_srv_.response.message.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service motion_ctrl");
            return 1;
        }
        return 0;
    }

    int VideoRobot::moveLineb(std::vector<float> pose_)
{
    setMode(0);
    setState(0);
    xarm_msgs::Move srv;
    srv.request.mmvelo = 100;
    srv.request.mvacc = 1000;
    srv.request.mvtime = 0;
    srv.request.mvradii = 20;
    srv.request.pose = pose_;
//moveb需要一些条件
//      /xarm/wait_for_finish -->false!
//      sleep发送请求之前需要发布sleep_msgs
    nh.setParam("/xarm/wait_for_finish", false); 
    std_msgs::Float32 sleep_msg;
    sleep_msg.data = 1.0;
    sleep_pub_.publish(sleep_msg);

    if (move_lineb_client_.call(srv))
    {
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service move_lineb");
        return 1;
    }

    nh.setParam("/xarm/wait_for_finish", true);
    return 0;

}

    int VideoRobot::goMyHome()
    {
        if (motionEnable())
        {
            return 1;
        }
        //move_line需要
        if (setMode(0) || setState(0))
        {
            return 1;
        }
        
        std::vector<float> homevc;
        homevc.push_back(my_home0.position.x);
        homevc.push_back(my_home0.position.y);
        homevc.push_back(my_home0.position.z);
        homevc.push_back(my_home0.rotation.r);
        homevc.push_back(my_home0.rotation.p);
        homevc.push_back(my_home0.rotation.y);

        if (moveLineb(homevc))
        {
            ROS_INFO("GO HOME!\n");
        }
        else
        {
            return 1;
        }
        return 0;
    }
    
    int VideoRobot::goVideo()
    {
        
    }