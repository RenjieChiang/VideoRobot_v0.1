#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include<tf/tf.h>
// #include<xarm_api/include/xarm_driver.h>
#include <iostream>
#include <fstream>

const std::string filename="/home/tianbot/testcppcode/transdataTRUE";

int go_home(moveit::planning_interface::MoveGroupInterface& move_group);
//回到定义的home点
//输入：操作的movegroup类

bool ReadCameraData( std::string filename , std::vector<geometry_msgs::Pose>& waypoints);
//读取文本保持的相机数据
//输入：文件名、路径点（引用）


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //节点启动
  
  
  static const std::string PLANNING_GROUP = "xarm6";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//move_group
// moveit::planning_interface::MoveGroupInterfacePtr move_groupptr;
  // const moveit::core::JointModelGroup* joint_model_group =
  //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//关节组状态

std::string reference_name = "link_base";//规划坐标系设置
move_group.setPoseReferenceFrame(reference_name);

ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

ROS_INFO_NAMED("tutorial","pose frame %s",move_group.getPoseReferenceFrame().c_str());

  // We can also print the name of the end-effector link for this group.
ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
ROS_INFO_NAMED("tutorial", "Available Planning Groups:");

std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
//打印当前机器信息


//调用visualtool组件，创建步进按钮
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("xarm_link0");
visual_tools.deleteAllMarkers();
visual_tools.loadRemoteControl();//按钮控制
visual_tools.trigger();


bool HOME=go_home(move_group);
ROS_INFO_NAMED("TEST","go home %s",HOME?"success":"failed");

// moveit::core::RobotState start_state(*move_group.getCurrentState());
// move_group.setStartState(start_state);

// geometry_msgs::Pose target_pose1;
// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// bool success;

visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo ");
visual_tools.trigger();

double time = move_group.getPlanningTime();
ROS_INFO("%f",time);
move_group.setMaxAccelerationScalingFactor(1.0);
move_group.setMaxVelocityScalingFactor(1.0);
move_group.setGoalTolerance(0.003);
// tf::Quaternion q;
std::vector<geometry_msgs::Pose>waypoints;
bool success=ReadCameraData(filename,waypoints);

// target_pose1.position.x=0+0.35;
// target_pose1.position.y=0;
// target_pose1.position.z=0+0.25;
// q=tf::createQuaternionFromRPY(3.1415,0.0,0.0);//保持末端朝下的姿态
// target_pose1.orientation.w=q.getW();
// target_pose1.orientation.x=q.getX();
// target_pose1.orientation.y=q.getY();
// target_pose1.orientation.z=q.getZ();

// waypoints.push_back(target_pose1);

// geometry_msgs::Pose target_pose2;
// target_pose2.position.x=0.028+0.35;
// target_pose2.position.y=0.086;
// target_pose2.position.z=0+0.25;
// target_pose2.orientation=target_pose1.orientation;
// waypoints.push_back(target_pose2);

// geometry_msgs::Pose target_pose3;
// target_pose3.position.x=0.003+0.35;
// target_pose3.position.y=0.20;
// target_pose3.position.z=-0.04+0.25;
// target_pose3.orientation=target_pose1.orientation;
// waypoints.push_back(target_pose3);

// geometry_msgs::Pose target_pose4;
// target_pose4.position.x=-0.08+0.35;
// target_pose4.position.y=0.29;
// target_pose4.position.z=-0.07+0.25;
// target_pose4.orientation=target_pose1.orientation;
// waypoints.push_back(target_pose4);

// geometry_msgs::Pose target_pose5;
// target_pose5.position.x= -0.185+0.35;
// target_pose5.position.y= 0.37;
// target_pose5.position.z= -0.046+0.25;
// target_pose5.orientation=target_pose1.orientation;
// waypoints.push_back(target_pose5);

// geometry_msgs::Pose target_pose6;
// target_pose6.position.x= -0.24+0.35;
// target_pose6.position.y= 0.41;
// target_pose6.position.z= -0.021+0.25;
// target_pose6.orientation=target_pose1.orientation;
// waypoints.push_back(target_pose6);


// // moveit_msgs::RobotTrajectory trajectory;
// // const double jump_threshold=0.0;
// // const double eef_step=0.01;

// // double fraction = move_group.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
// // ROS_INFO_NAMED("tutorial","try %.2f%% acheived", fraction * 100.0);
// // visual_tools.trigger();

// // move_group.execute(trajectory);


// // geometry_msgs::Pose target_pose2;
// // target_pose2.position.x=0.2;
// // target_pose2.position.y=-0.2;
// // target_pose2.position.z=0.25;

// geometry_msgs::Pose target_pose;
// std::vector<geometry_msgs::Pose> waypoints1;

//  target_pose.position.x=0+0.45;
// target_pose.position.y=0;
// target_pose.position.z=0+0.35;
// q=tf::createQuaternionFromRPY(3.1415,0.0,0.0);//保持末端朝下的姿态
// target_pose.orientation.w=q.getW();
// target_pose.orientation.x=q.getX();
// target_pose.orientation.y=q.getY();
// target_pose.orientation.z=q.getZ();

// double centerA = target_pose.position.x;
// double centerB = target_pose.position.y;
//     double radius = 0.1;


//     for(double th=0.0; th<6.28; th=th+0.1)
//     {
// 		target_pose.position.x = centerA + radius * sin(th);
//         target_pose.position.y = centerB + radius * cos(th);
//         waypoints1.push_back(target_pose);
//     }
 
	// 笛卡尔空间下的路径规划
	// moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.0002;
	double fraction = 0.0;
  int maxtries = 100;   //最大尝试规划次数
  int attempts = 0;     //已经尝试规划次数
 
  while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        //通过插补规划一个Cartesian 路径，基于指定的一组路径点 waypoints, 
        //插补步长由 eef_step 指定，结果保存到 trajectory 中， 
        //参考系由 setPoseReferenceFrame() 设置； jump_threshold 设置跳跃阈值，
        //防止IK逆运动求解中的 jump； avoid_collisions 设为 true 禁止碰撞；
        //返回值为 0.0 到 1.0 的一个值表示成功规划得到的路径占全部路径的比例。 错误返回 -1.0


        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
 
	    
	    
 
	    // 执行运动
	    move_group.execute(trajectory);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
visual_tools.trigger();


go_home(move_group);



// move_group.setPoseTarget(target_pose1);
// // ROS_INFO_NAMED("TEST","TARGETpose%f",move_group.getPoseTarget().pose.orientation.x);

// //执行
// success=(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
// // try plan and execute
// ROS_INFO_NAMED("TEST", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
// move_group.execute(my_plan);




ros::shutdown();
return 0;
}



int go_home( moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<double> joint_group_positions;
  double a[]={0,0,-1.57079,0,1.57079,0};
  joint_group_positions.insert(joint_group_positions.begin(),a,a+6);//弧度

  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("mytest","go home %s",success?"":"failed");
  move_group.execute(my_plan);//规划，报错设置，执行
  
  return 1;
}

bool ReadCameraData( std::string filename , std::vector<geometry_msgs::Pose>& waypoints)
{
   double num[200][10];
   std::ifstream myfile;
   std::cout<<"opening data: "<<filename<<"\n";
   myfile.open(filename,std::ios::in);
  if(!myfile.is_open())
  {
    std::cout<<"failed to open file!";
    return 0;
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
    geometry_msgs::Pose target_pose;
    tf::Quaternion q;
    for (i = 0; i < count; i++)
    {
      target_pose.position.x=0.01*num[i][3]+0.35;
      target_pose.position.y=0.01*num[i][4];
      target_pose.position.z=0.01*num[i][5]+0.25;
      q=tf::createQuaternionFromRPY(3.14159,0,num[i][1]/180*3.14);
      target_pose.orientation.w=q.getW();
      target_pose.orientation.x=q.getX();
      target_pose.orientation.y=q.getY();
      target_pose.orientation.z=q.getZ();
      waypoints.push_back(target_pose);
    }
    return 1;
////
////
// std::vector<geometry_msgs::Pose> waypoints;
// ReadCameraData(waypoints);
// float pose[5];
// for (int i = 0; i < waypoints.size(); i++)
// {
//            pose[0]=waypoints[i].position.x;
//            pose[1]=waypoints[i].position.y;
//            pose[2]=waypoints[i].position.z;
//            pose[3]=waypoints[i].orientation.x;
//            pose[4]=waypoints[i].orientation.y;
//            pose[5]=waypoints[i].orientation.z;
//             srv.request.pose = pose;
//             if(client.call(srv))
//             {
//                 ROS_INFO("%s\n", srv.response.message.c_str());
//             }
//             else
//             {
//                 ROS_ERROR("Failed to call service move_lineb");
//                 return 1;
//             }
//         }
// }


}