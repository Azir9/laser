/*
7_20 ubuntu 
7_21 debug under ubuntu
7_22 发布坐标
*/
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "robot.h"
#include "math.h"
#include "follower_server/robot_msg.h"
#include "iostream"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "ctime"



using namespace std;

clock_t start,end_time;

class pika
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

class Laser 
{
  public:
  	ros::Subscriber sub_scan;
   	int idnwis;
   	int scan_len;
   	sensor_msgs::LaserScan scan_data;
  	bool first_scan;
    sensor_msgs::LaserScan last_scan_data;
	float use_data[300];
	float min_cluster_num;
	float cluster_min_max_disus;
	ros::Publisher pose_pub;
	ros::Publisher markers_pub_;
	bool get_first;
	float first_ob1_x;
	float first_ob1_y;
	float success_num =0;
	float frame_num =0; 

	std::vector<pika> cluster(float ranges[],int len);// 雷达聚类函数
	
	float max_cluster_dis;//最大聚类距离，如果大于相邻两个激光点的距离大于max_cluster_dis，就认为属于不同的类



  	ros::NodeHandle nh;//创建句柄
  	Laser():idnwis(0),scan_len(0),first_scan(true),max_cluster_dis(0.0035),min_cluster_num(3),
	  cluster_min_max_disus(0.2)
  	{
    	sub_scan = nh.subscribe("/scan_filtered", 1, &Laser::scan_callback, this);
		pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ob_pose", 20);

		markers_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
  	}

  	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);

	void Pub_Mark(float x, float y, int id, float time);
	float distence(pika a,pika b);
};


  void ToCartesian(int i,float dis, float &x,float &y)
  {
    float Angle = i*0.5*3.14159/360.0+0.25*3.14159;
    x = dis*(sin(Angle));
    y = -dis*(cos(Angle));//转到二位迪卡尔坐标系
  }

  // 转到二维迪卡尔坐标系

void Laser::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	start = clock();
	//openni_stop = 0;
	//openni_start = 0;
    //std::unique_lock<std::mutex> lock(scan_mutex);
	printf("lock mutex");
/*对雷达数据长度进行计算*/
	if (scan_len == 0)
	{
		scan_len = scan->ranges.size();
		printf("scan_len:%d\n", scan_len);
	}
/*
	对数值进行赋值
*/
	scan_data.ranges.resize(scan_len);
	scan_data.ranges = scan->ranges;
	scan_data.header.frame_id = scan->header.frame_id;
	scan_data.angle_min = scan->angle_min;
	scan_data.angle_max = scan->angle_max;
	scan_data.angle_increment = scan->angle_increment;
	scan_data.intensities = scan->intensities;
	//            printf("angle_increment:%d\n",scan_data.angle_increment);

	scan_data.time_increment = scan->time_increment;
	scan_data.scan_time = scan->scan_time;
	scan_data.range_min = scan->range_min;
	scan_data.range_max = scan->range_max;
	scan_data.time_increment = scan->time_increment;
	scan_data.scan_time = scan->scan_time;
	scan_data.range_min = scan->range_min;
	scan_data.range_max = scan->range_max;

	scan_data.header.stamp = scan->header.stamp;
	printf("laser data compete");
	
	if (first_scan) 
	{

		last_scan_data.ranges.resize(scan_len);
		last_scan_data.ranges = scan->ranges;
		last_scan_data.header.frame_id = scan->header.frame_id;
		last_scan_data.angle_min = scan->angle_min;
		last_scan_data.angle_max = scan->angle_max;
		last_scan_data.angle_increment = scan->angle_increment;
		last_scan_data.intensities = scan->intensities;
		//            printf("angle_increment:%d\n",scan_data.angle_increment);

		last_scan_data.time_increment = scan->time_increment;
		last_scan_data.header.stamp = scan->header.stamp;
		last_scan_data.scan_time = scan->scan_time;
		last_scan_data.range_min = scan->range_min;
		last_scan_data.range_max = scan->range_max;

		first_scan = false;
	}

/*

	数据优化

*/

	for (int i = 0; i < scan_len; i++)
	{
		if (std::isnan(scan_data.ranges[i]))
		{
			use_data[i] = 0.0;
		}
		else
		{
			use_data[i] = scan_data.ranges[i];
		}
	}
	printf("cluster start\n");


	std::vector<pika> vpika = cluster(use_data,scan_len);
	
}
/*
下面函数进行聚类
*/

std::vector<pika> Laser::cluster(float ranges[], int len)
{
	std::vector<pika> vpika;//建立一个空对象
	pika pika_; //实例化 
	pika_.candidate = true;
	int cluster_index = 0;


	for (int j = 0; j < len; j++)
	{
		pika_.start_index = j;
		pika_.min_dis = ranges[j];
		pika_.max_dis = ranges[j];
		float all_x, all_y, dis;
		ToCartesian(j, ranges[j], all_x, all_y);//把点都转到极坐标系
		float all_dis = ranges[j];
		float clu_len = 0;
		while (std::fabs(ranges[j] - ranges[j + 1]) < max_cluster_dis)//对副点数求绝对值，阈值估计
		{
			// 当相邻两点的距离在阈值之内的话
			// 
			if (ranges[j + 1] > pika_.max_dis)//聚类之后，最右面那个点
				pika_.max_dis = ranges[j + 1];

			// 聚类之后最左面那个点

			if (ranges[j + 1] < pika_.min_dis)
				pika_.min_dis = ranges[j + 1];
			
			// 

			float temp_x, temp_y;
			ToCartesian(j + 1, ranges[j + 1], temp_x, temp_y);

			// 

			all_x = all_x + temp_x;
			all_y = all_y + temp_y;



			all_dis = all_dis + ranges[j + 1];
			//printf("rangs:%f,x:%f,y:%f\n",ranges[i+1],temp_x,temp_y);
			j = j + 1;
		}

	pika_.end_index = j; //赋值最后一个位置的点j;

	pika_.avg_dis = all_dis / (pika_.end_index - pika_.start_index + 1);//点之间的平均记录

	pika_.avg_x   = all_x   / (pika_.end_index - pika_.start_index + 1);//聚类之后的x
	
	pika_.avg_y   = all_y   / (pika_.end_index - pika_.start_index + 1);//聚类之后的y
	
		for (int aa = pika_.start_index; aa < pika_.end_index; aa++)
		{

		
			float temp_x, temp_y;
			ToCartesian(aa + 1, ranges[aa + 1], temp_x, temp_y);
		
			float temp_x1, temp_y1;
			ToCartesian(aa, ranges[aa], temp_x1, temp_y1);

			dis = sqrt((temp_x - temp_x1) * (temp_x - temp_x1) + (temp_y - temp_y1) * (temp_y - temp_y1));
			
			clu_len = clu_len + dis;
		}
	}
	//聚类完成
/*
	进行聚类之后的数据比较
*/
	bool is_ob_num_ = (pika_.end_index - pika_.start_index) > min_cluster_num;//至少是多少个点
	bool is_ob_dis_ = std::fabs(pika_.min_dis - pika_.max_dis) < cluster_min_max_disus;
 
	//----------------可以再加几个判断条件----------------------------！！//

	//曲率片段
/*
	数据汇总
*/
	std::vector<pika> v_ob_;
	printf("begin find obsition");
	for (auto it = vpika.begin(),ite = vpika.end(); it!=ite;++it)
	{
		if(it!=ite-1)
      {
        for(auto next_cluster= it+1, ite_= vpika.end(); next_cluster!=ite_; ++next_cluster)
        {	
		  /* 判断代码，后期要改

          if(it->avg_x>max_first || it->avg_x<min_first || std::fabs(it->avg_y)>0.5)//左右0.4的范围可适当调整
                continue;
          //printf("condition---01\n");

          if(next_cluster->avg_x>max_first || next_cluster->avg_x<min_first || std::fabs(next_cluster->avg_y)>0.5)//左右0.4的范围可适当调整
                continue;
          //printf("condition---02\n");
          //printf("02,next x:%f,y:%f\n",next_leg_cluster->avg_x,next_leg_cluster->avg_y);
          
          //纵向距离差要小于0.35
          if(std::fabs(it->avg_x - next_cluster->avg_x)>0.4)
                continue;
          //printf("condition---03\n");
          //printf("031,next x:%f,y:%f\n",next_leg_cluster->avg_x,next_leg_cluster->avg_y);
          //相邻两个聚类的距离小于0.5m，且他们的长度差小于0.2,则找到初始人的位置
		  */

		  
          if(distence((*it),(*next_cluster)) < 0.5 ||std::fabs(it->length - next_cluster->length)<0.3)
          {
                v_ob_.push_back(*it);
                v_ob_.push_back(*next_cluster);
                get_first =true;

                first_ob1_x = v_ob_[0].avg_x;
                first_ob1_y = v_ob_[0].avg_y;

                success_num++;
                frame_num++;

                break;
				int index = 1;
				Pub_Mark(first_ob1_x,first_ob1_y,index,10.0);
          }
        //end for*/
		
        if(get_first)
		{
          break;
		}
		}
	  }

	}

}
float Laser::distence(pika a,pika b)
{
  return std::sqrt((a.avg_x-b.avg_x)*(a.avg_x-b.avg_x)+(a.avg_y-b.avg_y)*(a.avg_y-b.avg_y));
}
void Laser::Pub_Mark(float x, float y, int id, float time)
{
				visualization_msgs::Marker m;
				m.header.stamp = ros::Time::now();
				m.header.frame_id = "laser";
				m.ns = "LEGS";
				m.id = id;
				m.type = m.SPHERE;
				m.pose.position.x = x;
				m.pose.position.y = y;
				m.pose.position.z = 0;

				m.scale.x = .1;
				m.scale.y = .1;
				m.scale.z = .1;
				m.color.a = 1;
				m.lifetime = ros::Duration(time);
			    m.color.b = 1;

        	    markers_pub_.publish(m);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_Clustering");
	ros::NodeHandle nh;

	ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);
	int count = 0;

	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "how are you " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
