/*
7_20 ubuntu无线网卡坏了，win简单写代码

*/
#include "ros/ros.h" 
#include "std_msgs/String.h" 

#include <sstream>



using namespace std;
class Laser 
{
	public:
		ros::Subscriber sub_scan;

	ros::NodeHandle nh;
	Laser():
	{
		sub_scan = nh.subscribe("/scan_filtered", 1, &Laser::scan_callback, this);
	}
}

void Laser::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	start = clock();
	openni_stop = 0;
	openni_start = 0;
	std::unique_lock<std::mutex> lock(scan_mutex);
	printf("lock mutex");

	if (scan_len == 0)
	{
		scan_len = scan->ranges.size();
		printf("scan_len:%d\n", scan_len);
	}
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
	
	if (first_scan) {

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

	for (int i = 0; i < scan_lan; i++)
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
}
/*
进行聚类算法的函数
*/
std::vector<pika> Robot::cluster(float ranges[], int len)
{
	std::vector<pika> vpika;
	pika pika_;
	pika_.candidate = true;
	int cluster_index = 0;


	for (int j = 0; j < len; j++)
	{
		pika_.start_index = j;
		pika_.min_dis = ranges[j];
		pika_.max_dis = ranges[j];
		float all_x, all_y, dis;
		ToCartesian(j, ranges[j], all_x, all_y);//数据转到笛卡尔坐标系
		float all_dis = ranges[i];
		float clu_len = 0;
		while (std::fabs(ranges[j] - ranges[j + 1]) < max_cluster_dis)//相邻两个激光距离小于阈值max_cluster_dis，为同一类
		{
			//更新最大距离
			if (ranges[j + 1] > pika_.max_dis)
				pika_.max_dis = ranges[j + 1];

			//更新最小距离
			if (ranges[j + 1] < pika_.min_dis)
				pika_.min_dis = ranges[j + 1];

			float temp_x, temp_y;
			ToCartesian(j + 1, ranges[j + 1], temp_x, temp_y);

			all_x = all_x + temp_x;
			all_y = all_y + temp_y;



			all_dis = all_dis + ranges[j + 1];
			//printf("rangs:%f,x:%f,y:%f\n",ranges[i+1],temp_x,temp_y);
			j = j + 1;
		}
	}
	pika_.end_index = j;
	pika_.avg_dis = all_dis / (pika_.end_index - pika_.start_index + 1);//这一类点的平均距离

	pika_.avg_x = all_x / (pika_.end_index - pika_.start_index + 1);//这一类点的平均X
	pika_.avg_y = all_y / (pika_.end_index - pika_.start_index + 1); //这一类点的平均Y
	for (int aa = Leg_cluster_.start_index; aa < Leg_cluster_.end_index; aa++)
	{
		float temp_x, temp_y;
		ToCartesian(aa + 1, ranges[aa + 1], temp_x, temp_y);
		float temp_x1, temp_y1;
		ToCartesian(aa, ranges[aa], temp_x1, temp_y1);

		dis = sqrt((temp_x - temp_x1)*(temp_x - temp_x1) + (temp_y - temp_y1)*(temp_y - temp_y1));
		clu_len = clu_len + dis;

	}
/*桶堆判断*/
/*发布数据*/

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