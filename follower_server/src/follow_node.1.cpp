#include "robot.h"
#include <boost/thread/thread.hpp>
/*
konanrobot@163.com By YEL
*/

#define PUB_ONCE

int main(int argc, char **argv)  
{  
  //初始化ROS  
  ros::init(argc, argv, "follow_node");  
  
  //类实例化
  Robot robot;
  ros::Rate loop_rate = 5;
  ros::NodeHandle n;
  //利用类成员函数定义服务回调函数
  ros::ServiceServer follow_service = n.advertiseService("follow", &Robot::follower, &robot);

while (ros::ok())
{
    //printf("9999\n");
  if(robot.openni_start)
  {
            printf("start openni\n");
          robot.openni_order.data = "start";
          robot.openni_pub_.publish(robot.openni_order);
          loop_rate.sleep();
  }
  if(robot.openni_stop)
  {
                printf("stop openni\n");
          robot.openni_order.data = "stop";
          robot.openni_pub_.publish(robot.openni_order);
          loop_rate.sleep();

  }

  ros::spinOnce();
  loop_rate.sleep();
}
//  ros::spin();

  return 0;  
}  

