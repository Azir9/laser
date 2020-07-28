#include <tf/transform_listener.h>
#include "ros/ros.h"  
#include <move_base_msgs/MoveBaseAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GetAMCLPose");//初始化节点
  tf::TransformListener listener;
tf::StampedTransform transform;
  while(ros::ok())
  {
    try
    {
        //ROS_INFO("Attempting to read pose...");
        listener.lookupTransform("/map","/base_link",ros::Time(0), transform);

        ROS_INFO("当前机器人位置 x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
        //myfile << transform.getOrigin().x() << "," << transform.getOrigin().y() << "\n";
        //tf::Quaternion tfq = transform.getRotation();
        //move_base_msgs::MoveBaseGoal goal_cash;
        //quaternionTFToMsg(tfq, goal_cash.target_pose.pose.orientation);
        //float angle = tf::createYawFromQuaternionMsg(goal_cash.target_pose.pose.orientation);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Nope! %s", ex.what());
    }
  }
  return 1;
}
