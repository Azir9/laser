#include "robot.h"
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iomanip>
/*
konanrobot@163.com By YEL
*/

#define PUB_ONCE

using namespace std;


int main(int argc, char **argv)  
{  
  //初始化ROS  
  ros::init(argc, argv, "follow_node");  
  
  //类实例化
  Robot robot;
  ros::Rate loop_rate = 20;
  ros::NodeHandle n;
  //利用类成员函数定义服务回调函数
  ros::ServiceServer follow_service = n.advertiseService("follow", &Robot::follower, &robot);
    //创建文件



while (ros::ok())
{


  ros::spinOnce();
  loop_rate.sleep();
}

  return 0;  
}  

