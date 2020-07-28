#include "ros/ros.h"
#include "std_msgs/String.h"
#include "follower_server/follow_srv.h"
#include <sstream>
/*******************leg detecter***********************/
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h> 
#include <vector>
#include <iostream>
#include <mutex>
#include <tf/transform_listener.h>
#include <Eigen/Core>
//#include "openni_tracker_test/SetFollowState.h"

#define TEXT_NORMAL "\033[0m"
#define TEXT_RED   "\033[1;31m"
#define TEXT_GREEN "\033[1;32m"
#define TEXT_PINK "\033[1;35m"
/*
konanrobot@163.com By YEL
*/
class Leg_cluster
{
public: 
  int start_index;
  int end_index;
  float avg_dis;//当前聚类的所有点与激光的平均距离
  float min_dis;//当前聚类中与激光的最小距离
  float max_dis;//当前聚类中与激光的最大距离
  float avg_x;//聚类中心的x坐标
  float avg_y;//聚类中心的y坐标
  float length;//该聚类的长度
  bool candidate;//是否是候选者，默认是
  int index;//该聚类的索引号
  float dis_last;//该聚类与上一个人位置的距离
  float sum_length;//该聚类总长度
  float curve;//聚类的曲率
  int min_dis_index;
};

bool Leg_Candidate_Sort(const Leg_cluster &a, const Leg_cluster &b)
{
	return a.dis_last < b.dis_last;
}

float distence(Leg_cluster a,Leg_cluster b)
{
  return std::sqrt((a.avg_x-b.avg_x)*(a.avg_x-b.avg_x)+(a.avg_y-b.avg_y)*(a.avg_y-b.avg_y));
}

float distence_(Leg_cluster a, float x, float y)
{
  return std::sqrt((a.avg_x-x)*(a.avg_x-x)+(a.avg_y-y)*(a.avg_y-y));
}


class People
{
public: 
  float x;
  float y;
};

class Robot  
{  
public:  
  Robot(void); 

/*******************leg detecter***********************/
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool follower(follower_server::follow_srv::Request  &req,follower_server::follow_srv::Response &res);
  //激光数据聚类算法。ranges为需要聚类的激光数据数组，len为该数组的大小。返回值为聚类向量
  std::vector<Leg_cluster> cluster(float ranges[] , int len);
  std::vector<Leg_cluster> Cull_vLeg(std::vector<Leg_cluster>);
  std::vector<People> LegToPeople(std::vector<Leg_cluster> vLeg_Cluster);

void Pub_Leg_Mark(float x, float y, int id, float time=0.3);
void Pub_Text_Mark(float x, float y, int id, std::string title, float time=0.3);
void Pub_People_Mark(float x, float y, int id, float time=0.3);
void Pub_Speed();
/*******************leg detecter***********************/
  //第i个激光数据，障碍物距离激光dis，得到笛卡尔空间下的坐标x，y
  void ToCartesian(int i,float dis, float &x,float &y)
  {
    float Angle = i*0.5*3.14159/360.0+0.25*3.14159;
    x = dis*(sin(Angle));
    y = -dis*(cos(Angle));//对激光数据进行分解
  } 
  //为该进程的节点创建一个句柄。第一个创建的NodeHandle会为节点进行初始化，最后一个销毁的会清理节点使用的所有资源。
  ros::NodeHandle n_;   

  //订阅类 
  ros::Subscriber sub_; 
  //ros::Rate对象可以允许你指定自循环的频率。它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间。程序运行频率(Hz)
  ros::Rate loop_rate_;
/*******************leg detecter***********************/
  std::mutex scan_mutex;
  People people_;
  float keep_dist;
 
  sensor_msgs::LaserScan scan_data;
  float Scan_Data[897];
  //订阅类 
  ros::Subscriber sub_scan_; 
  ros::Publisher markers_pub_;
  ros::Publisher speed_pub_;
  ros::Publisher openni_pub_;
  bool pub_leg_mark;
  bool pub_people_mark;

  bool start_follow;//是否开始跟踪
  bool stop_follow;//是否停止跟踪
  bool processing_scan;//是否正在处理激光数据
  bool first_data;//是否是第一帧数据
  bool first_data_sucess;//第一帧是否找人成功

  float min_first;//第一帧人距离激光的最小距离
  float max_first;//第一帧人距离激光的最大距离

  float first_leg1_x;
  float first_leg1_y;
  float first_leg2_x;
  float first_leg2_y;

  float last_x;//上一帧人的xy坐标
  float last_y;

  bool set_use_predict_vel;//是否使用预测速度来跟踪
  bool get_predict_vel;//使用预测速度预测当前帧的位置
  float predict_vel_x;//预测x方向的速度
  float predict_vel_y;//预测y方向的速度

  int scan_len;//激光数据长度。velodyne激光共897个数据
  float max_cluster_dis;//最大聚类距离，如果大于相邻两个激光点的距离大于max_cluster_dis，就认为属于不同的类
  int min_cluster_num;//每个类包含的最小激光点数目
  float cluster_min_max_dis;//同一类中，点到激光的最大最小距离应该在一个范围之内
  float max_Leg_range;//聚类时，类里面的平均距离应小于的最大距离

  int count;
  int start_count;
  int stop_count;

  float max_linear_vel;
  float max_angular_vel;
  geometry_msgs::Twist vel_cmd;
  std_msgs::String openni_order;
  bool openni_start;
  bool openni_stop;
  bool follow_finish;

  float max_curve;
  float min_curve;

  //bool tracker_state;
/*******************leg detecter***********************/
};
