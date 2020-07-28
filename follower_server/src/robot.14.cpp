/*
2020.1.9 删除openni相关的代码
2020.1.23 修改雷达数据话题 line497
2020.2.14 z增加曲率判断条件 line24 line233
2020.2.28 写数据到文件  line61
2020.3.3  加入卡尔曼滤波
2020.3.6  调试卡尔曼滤波功能
2020.3.15 记录两组有无卡尔曼滤波的数据
2020.3.16 考虑行人丢失的情况
2020.3.19 加入帧差法
2020.3.26 检测成功率
2020.4.4 考虑解决将障碍物误认为行人的问题
*/
#include "robot.h"
#include "follower_server/robot_msg.h"
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include<ctime>
/*
konanrobot@163.com By YEL
*/
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
clock_t start,end_time;

//类的构造函数
Robot::Robot():loop_rate_(10),start_follow(false),stop_follow(false),processing_scan(false),first_data(true),scan_len(0),
min_first(0.2),max_first(5.0),
first_data_sucess(false),
max_cluster_dis(0.035),min_cluster_num(4),cluster_min_max_dis(0.25),
max_Leg_range(5),
pub_leg_mark(true),pub_people_mark(true),
predict_vel_x(0.0),predict_vel_y(0.0),get_predict_vel(0),set_use_predict_vel(true),
last_x(0.0),last_y(0.0),
keep_dist(1.0),
max_angular_vel(0.6),max_linear_vel(0.55),start_count(0),stop_count(0),count(0),openni_start(0),openni_stop(1),follow_finish(0),
max_curve(1.35),min_curve(1.06),update_n(3),first_scan(1),use_diff(false),diff_num(3),lost_people(false)
  {  
    //获取参数
    /*n_.param("/robot_node/speakwords", words_, std::string("hello world"));
    n_.param<float>("/robot_node/min_first", min_first, 0.4);
    n_.param<float>("/robot_node/max_first", max_first, 1.8);
    n_.param<float>("/robot_node/keep_dist", keep_dist, 0.8);
    n_.param<float>("/robot_node/max_angular_vel", max_angular_vel, 1.5);
    n_.param<float>("/robot_node/max_linear_vel", max_linear_vel, 0.5);

    n_.param<float>("/robot_node/max_cluster_dis", max_cluster_dis, 0.1);
    n_.param<int>("/robot_node/min_cluster_num", min_cluster_num, 3);
    n_.param<float>("/robot_node/cluster_min_max_dis", cluster_min_max_dis, 0.3);
    */

    //需要订阅的话题
    //告诉master我们要订阅robot_speaker topic上的消息。当有消息到达topic时，ROS就会调用Robot::callback()函数。第二个参数是队列大小。 

    sub_scan_ = n_.subscribe("/scan_filtered", 1, &Robot::scan_callback, this);  

    // sub_scan_ = n_.subscribe("/scan", 1, &Robot::scan_callback, this);  

    markers_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    speed_pub_ = n_.advertise<geometry_msgs::Twist>("raw_cmd_vel", 20);

    scan_pub = n_.advertise<sensor_msgs::LaserScan>("diff_scan", 50);
    //openni_pub_ = n_.advertise<std_msgs::String>("/openni_follow",1);
		    x_ = VectorXd(4);
      z_ = VectorXd(2);
	    F_ = MatrixXd(4, 4);
      P_ = MatrixXd(4, 4);
      R_laser_ = MatrixXd(2, 2);
      H_laser_ = MatrixXd(2, 4);
	    Q_ = MatrixXd(4, 4);
      R_laser_ << 0.0225, 0,
      0, 0.0225;
      
      H_laser_<<1,0,0,0,
            0,1,0,0;

      //state covariance matrix P
      P_ << 1, 0, 0, 0,
      		  0, 1, 0, 0,
       		  0, 0, 10, 0,
       		  0, 0, 0, 10;
    
      //the initial transition matrix F_ ?predict time?
      F_ << 1, 0, 1, 0,
       		  0, 1, 0, 1,
       		  0, 0, 1, 0,
       		  0, 0, 0, 1;
       		  
	    // Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			//    0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			//    dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			//    0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
	    Q_ <<  1/4*noise_ax, 0, 1/2*noise_ax, 0,
			   0, 1/4*noise_ay, 0, 1/2*noise_ay,
			   1/2*noise_ax, 0, 1*noise_ax, 0,
			   0, 1/2*noise_ay, 0, 1*noise_ay;

      x_<<0,0,0,0;
  }   


void Robot::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)  
  { 
    start=clock();
    openni_stop = 0;
    openni_start = 0;
    std::unique_lock<std::mutex> lock(scan_mutex);
    printf("1111\n");
//写数据到文件
     FILE  *outfile;
	  outfile=fopen("kalman.txt","a");
    //初始化scan_data
    if(scan_len == 0)
    {
      scan_len = scan->ranges.size();
      printf("scan_len:%d\n",scan_len);
    }

      scan_data.ranges.resize(scan_len);
      scan_data.ranges=scan->ranges;
      scan_data.header.frame_id = scan->header.frame_id;
      scan_data.angle_min = scan->angle_min;
      scan_data.angle_max = scan->angle_max;
      scan_data.angle_increment = scan->angle_increment;
      scan_data.intensities=scan->intensities;
        //            printf("angle_increment:%d\n",scan_data.angle_increment);

      scan_data.time_increment = scan->time_increment;
      scan_data.scan_time = scan->scan_time;
      scan_data.range_min= scan->range_min;
      scan_data.range_max= scan->range_max;

    scan_data.header.stamp = scan->header.stamp;
        printf("2222\n");

      if(first_scan){
       
          last_scan_data.ranges.resize(scan_len);
          last_scan_data.ranges=scan->ranges;
          last_scan_data.header.frame_id = scan->header.frame_id;
          last_scan_data.angle_min = scan->angle_min;
          last_scan_data.angle_max = scan->angle_max;
          last_scan_data.angle_increment = scan->angle_increment;
          last_scan_data.intensities=scan->intensities;
            //            printf("angle_increment:%d\n",scan_data.angle_increment);

        last_scan_data.time_increment = scan->time_increment;
        last_scan_data.header.stamp = scan->header.stamp;
        last_scan_data.scan_time = scan->scan_time;
        last_scan_data.range_min= scan->range_min;
        last_scan_data.range_max= scan->range_max;
        
        first_scan=false;
      }
    //laser_diff
    //   if((use_diff||lost_people||first_data)){
    // //if((use_diff)){
    //   if(diff_count>5){
    //             printf("----------2222\n");
    //     diff_count=0;
    //     diff_data[897]={0};
    //   }
    //   Frame_Diff(scan);

    //  }

    // for(int i= scan_len/4;i<scan_len*3/4;++i)
    for(int i= 0;i<scan_len;++i)
    {
            //printf("%d\n",i);

        // if(std::isnan(scan->ranges[i]))
        // {
        //   Scan_Data[i] = 0.0;
        // }
        // else
        // {
        //   Scan_Data[i] = scan->ranges[i];
        //     //找到最近的障碍点
			
        // }
                if(std::isnan(scan_data.ranges[i]))
        {
          Scan_Data[i] = 0.0;
        }
        else
        {
          Scan_Data[i] = scan_data.ranges[i];
            //找到最近的障碍点
			
        }
    }
	  
	    //对激光数据进行聚类，得到可能的Leg位置
		printf("cluster start\n");//开始聚类
		std::vector<Leg_cluster> vLeg_Cluster = cluster(Scan_Data,scan_len);

    //根据Leg聚类，解析People的位置
    //printf("LegToPeople start\n");
    std::vector<People> vPeople_Cluster;  //人腿位置放在vPeople_Cluster向量里
  //发送跟随速度 
  if(first_data_sucess && (!lost_people))
  {
    frame_num++;
    vPeople_Cluster = LegToPeople(vLeg_Cluster);
    printf("------success_num:%.0f,frame_num:%.0f,success percent:%f------\n",success_num,frame_num,success_num/frame_num);//
  }

      if( !vPeople_Cluster.size() && !lost_people &&first_data_sucess)
      {
        if( lost_count<15)
        {
           lost_count++;
        }

        else
        {
          lost_people=true;
          printf("---------Lost poeple in laser!--------\n");
          lost_count = 0;
        }
      }
      else if(montionless_flag && !lost_people &&first_data_sucess)
      {
        montionless_flag = false;
        if( lost_count<25)
        {
           lost_count++;
        }

        else
        {
          lost_people=true;
          printf("---------Lost poeple in laser!--------\n");
          lost_count = 0;
        }
      }
      else
      {
          lost_count = 0;
      }

    if(start_follow && !follow_finish)
    {
      if(!vPeople_Cluster.size())
      {
        if(start_count<20)
        {
           start_count++;
        }
        else
        {
          //vel_cmd.linear.x = 0;
          //vel_cmd.angular.z = 0; 
          openni_stop = 0;
          openni_start = 1;
          printf("---------Lost poeple in laser!--------\n");
          start_count = 0;
          //start openni_tracker
          //openni_srv.request.state=1;
          //if(openni_client.call(openni_srv))          
            //printf("start openni_tracker\n");
        }
      }
      else
      {
        if(stop_count<5)
        {
           stop_count++;
        }
        else
        {
        //stop openni_tracker
          openni_stop = 1;
          openni_start = 0;
          stop_count = 0;

        }
          //if(openni_client.call(openni_srv))
                      //printf("stop openni_tracker\n");
          Pub_Speed();
      }
      speed_pub_.publish(vel_cmd);

    }
    else
    {
          openni_stop = 0;
          openni_start = 0;
        if(count<5 && !openni_start && !openni_stop)
        {
           count++;
          // vel_cmd.linear.x = 0;
          // vel_cmd.angular.z = 0;
          // speed_pub_.publish(vel_cmd);
        }
    }

      
    // if(vPeople_Cluster.size()){
    // printf("start write people data to file\n");
    // fprintf(outfile,"%.3f\t%.3f\n", last_x,last_y);
    end_time= clock();
    double last_time=(double)(end_time-start)/CLOCKS_PER_SEC;
    printf("TIME:%lf\n",last_time);
    // }
//  ros::spin();
  //fclose(outfile);
     
  }  

std::vector<Leg_cluster> Robot::cluster(float ranges[] , int len)
{

  std::vector<Leg_cluster> vLeg_Cluster;
  Leg_cluster Leg_cluster_;
  Leg_cluster_.candidate = true;
  int cluster_index=0;
/*1.统计聚类数据*/
  //printf("统计聚类数据\n");
  // for(int i=len/4;i<len*3/4;++i)
  for(int i=0;i<len;++i)
  {
    Leg_cluster_.start_index = i;
    Leg_cluster_.min_dis = ranges[i];
    Leg_cluster_.max_dis = ranges[i];
    float all_x,all_y,dis;
    ToCartesian(i,ranges[i],all_x,all_y);

    float all_dis = ranges[i];
    float clu_len = 0;
    //printf("-------------------------------------------------------------\n");
    //聚类方式暂定，之后会修改。
    while(std::fabs(ranges[i]-ranges[i+1])< max_cluster_dis )//相邻两个激光距离小于阈值max_cluster_dis，为同一类
    {
	      //更新最大距离
	      if(ranges[i+1] > Leg_cluster_.max_dis)
		    Leg_cluster_.max_dis = ranges[i+1];

	      //更新最小距离
	      if(ranges[i+1] < Leg_cluster_.min_dis)
		    Leg_cluster_.min_dis = ranges[i+1];
	   
	      float temp_x,temp_y;
	      ToCartesian(i+1,ranges[i+1],temp_x,temp_y);

	      all_x = all_x+temp_x;
	      all_y = all_y+temp_y;



	      all_dis = all_dis + ranges[i+1];
        //printf("rangs:%f,x:%f,y:%f\n",ranges[i+1],temp_x,temp_y);
        i=i+1;
    }

    Leg_cluster_.end_index = i;
    Leg_cluster_.avg_dis = all_dis/(Leg_cluster_.end_index - Leg_cluster_.start_index +1);//这一类点的平均距离

    Leg_cluster_.avg_x = all_x/(Leg_cluster_.end_index - Leg_cluster_.start_index +1);//这一类点的平均X
    Leg_cluster_.avg_y = all_y/(Leg_cluster_.end_index - Leg_cluster_.start_index +1);//这一类点的平均Y
      
        //计算聚类总长
    for(int aa=Leg_cluster_.start_index;aa<Leg_cluster_.end_index;aa++)
    {
      	      float temp_x,temp_y;
	      ToCartesian(aa+1,ranges[aa+1],temp_x,temp_y);
	      float temp_x1,temp_y1;
	      ToCartesian(aa,ranges[aa],temp_x1,temp_y1);

        dis=sqrt((temp_x-temp_x1)*(temp_x-temp_x1)+(temp_y-temp_y1)*(temp_y-temp_y1));
        clu_len = clu_len+dis;

    }

/*2.Leg条件判断*/
    //2.1.如果该聚类有足够多的点数，则认为该聚类可能正确

    bool enough_point_ = (Leg_cluster_.end_index - Leg_cluster_.start_index) > min_cluster_num;//至少三个点
    //2.2.聚类的最大最小距离判断
    bool min_max_dis_ =  std::fabs(Leg_cluster_.min_dis - Leg_cluster_.max_dis) < cluster_min_max_dis;//这一类点中，距离最远的点和距离最近的点之间的差值小于一个阈值0.3？？？这么大
    //TODO::2.3.线段长度判断0.05-0.25m
    float start_x,start_y,end_x,end_y;
    ToCartesian(Leg_cluster_.start_index,ranges[Leg_cluster_.start_index],start_x,start_y);
    ToCartesian(Leg_cluster_.end_index,ranges[Leg_cluster_.end_index],end_x,end_y);
    float dis_ = std::sqrt((start_x-end_x)*(start_x-end_x)+(start_y-end_y)*(start_y-end_y));//这一类中起始点到最终点的距离不能超过某个阈值5到25厘米之间算是腿

    bool line_length = false;
    if(!std::isnan(dis_) && dis_>0.08 && dis_<0.21)//距离在5到35之间（随意给的，可改）
    {
      Leg_cluster_.length = dis_;
      line_length = true;
	  }

    //2.4.平均距离判断<2.5米，也可适当放大
    bool cluster_avg_dis = Leg_cluster_.avg_dis < max_Leg_range;//类里面的平均距离应小于max_Leg_range

    //2.5聚类曲率判断
    Leg_cluster_.sum_length = clu_len;
    Leg_cluster_.curve = clu_len/dis_;
    float curve = clu_len/dis_;
    bool cluster_curve = ((Leg_cluster_.curve > min_curve) && (Leg_cluster_.curve < max_curve));

     if(enough_point_ && min_max_dis_ && cluster_avg_dis && line_length && cluster_curve)//加入曲率条件
    //  if(enough_point_ && min_max_dis_ && cluster_avg_dis && line_length)
    {
      //printf("dis_:%f\n",dis_);
      Leg_cluster_.index = cluster_index;
      cluster_index = cluster_index+1;
      vLeg_Cluster.push_back(Leg_cluster_);
      printf("clu_len:%f,clu_dis:%f\n",clu_len,dis_);
    }


  }//end for

	/*
	for(auto it= vLeg_Cluster.begin(), ite= vLeg_Cluster.end(); it!=ite; ++it)
	{
		printf("index:%d,x:%f,y:%f\n",it->index,it->avg_x,it->avg_y);
	}
	*/

/*3.第一帧数据判断,得到唯一一对Leg（初始化）*/
  
  if(first_data||lost_people)
  {
    printf("初始Leg：［%d］\n",vLeg_Cluster.size());
    std::vector<Leg_cluster> vFirst_2_Leg;
    bool get_first =false;
	  for(auto it = vLeg_Cluster.begin(), ite= vLeg_Cluster.end(); it!=ite; ++it)
	  {
      if(it!=ite-1)
      {
        for(auto next_leg_cluster= it+1, ite_= vLeg_Cluster.end(); next_leg_cluster!=ite_; ++next_leg_cluster)
        {	
          //printf("cur index:%d, next index:%d\n",it->index,next_leg_cluster->index);
          //printf("first data--,cur x:%f,y:%f\n",it->avg_x,it->avg_y);
          //printf("first data--,next x:%f,y:%f\n",next_leg_cluster->avg_x,next_leg_cluster->avg_y);
          //当前聚类和下一个聚类都要满足距离条件
          if(it->avg_x>max_first || it->avg_x<min_first || std::fabs(it->avg_y)>0.5)//左右0.4的范围可适当调整
                continue;
          //printf("condition---01\n");

          if(next_leg_cluster->avg_x>max_first || next_leg_cluster->avg_x<min_first || std::fabs(next_leg_cluster->avg_y)>0.5)//左右0.4的范围可适当调整
                continue;
          //printf("condition---02\n");
          //printf("02,next x:%f,y:%f\n",next_leg_cluster->avg_x,next_leg_cluster->avg_y);
          
          //纵向距离差要小于0.35
          if(std::fabs(it->avg_x - next_leg_cluster->avg_x)>0.4)
                continue;
          //printf("condition---03\n");
          //printf("031,next x:%f,y:%f\n",next_leg_cluster->avg_x,next_leg_cluster->avg_y);
          //相邻两个聚类的距离小于0.5m，且他们的长度差小于0.2,则找到初始人的位置
          if( distence((*it),(*next_leg_cluster)) < 0.5 || std::fabs(it->length - next_leg_cluster->length)<0.3)
          {
                vFirst_2_Leg.push_back(*it);
                vFirst_2_Leg.push_back(*next_leg_cluster);
                get_first =true;

                first_leg1_x = vFirst_2_Leg[0].avg_x;
                first_leg1_y = vFirst_2_Leg[0].avg_y;
                first_leg2_x = vFirst_2_Leg[1].avg_x;
                first_leg2_y = vFirst_2_Leg[1].avg_y;
            
                last_x =(first_leg1_x+first_leg2_x)/2;
                last_y =(first_leg1_y+first_leg2_y)/2;

                last_per_x =last_x;
                last_per_y =last_y;

                success_num++;
                frame_num++;

                break;
          }
        }//end for

        if(get_first)
          break;
      }//end if(it!=ite-1)
	}//end for
    
  if(vFirst_2_Leg.size() == 2)
	{
    printf("初始化成功\n");
		first_data = false;
    first_data_sucess = true;
    lost_people=false;
    int first_2_legs_ID =200;
    for(auto it= vFirst_2_Leg.begin(), ite= vFirst_2_Leg.end(); it!=ite; ++it)
    {
      first_2_legs_ID =first_2_legs_ID+1;
			Pub_Leg_Mark(it->avg_x,it->avg_y,first_2_legs_ID,10.0);
		}//end for

    Pub_Text_Mark(last_x,last_y,110,"init people",10.0);

    return vFirst_2_Leg;
	}//end if(vFirst_2_Leg.size() == 2)

  }//end first data
/*4.第二帧及以后的Leg聚类处理*/
 

/*5.发布Leg Mark*/
  //printf("发布Leg Mark\n");
  if (pub_leg_mark)
  {
      int index = 1;
      for(auto it= vLeg_Cluster.begin(), ite= vLeg_Cluster.end(); it!=ite; ++it)
      {
        Pub_Leg_Mark(it->avg_x,it->avg_y,index);
        index = index+1;
      }
  }

  printf("Leg聚类个数：--[%d]--.\n",vLeg_Cluster.size());
  
  return vLeg_Cluster;
}

std::vector<People> Robot::LegToPeople(std::vector<Leg_cluster> vLeg_Cluster)
{
  //Leg To People
  std::vector<Leg_cluster> Leg_Candidate;
  std::vector<People> people_cluster;

  float predict_x = last_x;
  float predict_y = last_y;
  
 
  //使用速度预测，如果没有速度，就使用上一个people的位置
  if( set_use_predict_vel && get_predict_vel!=0)
  {
	predict_x = last_x + predict_vel_x;
	predict_y = last_y + predict_vel_y;
  }
  x_(0)=predict_x;
  x_(1)=predict_y;
  
  printf("last x:%f,last y:%f\n",last_x,last_y);
  printf("predict x:%f,predict y:%f\n",predict_x,predict_y);

  for(auto it= vLeg_Cluster.begin(), ite= vLeg_Cluster.end(); it!=ite; ++it)
  {
    if(it->candidate == false)
		continue;    
    it->dis_last = distence_(*it,predict_x,predict_y);

    //printf("leg x:%f,leg y:%f\n",it->avg_x,it->avg_y);
    //printf("dis:%f\n",dis_last);
    //当前leg聚类的中心与上一个人位置的距离是否小于0.3,如果小于，认为该leg聚类有极大可能是当前人的leg
    if(it->dis_last < 0.5 && it->avg_x >0.3)
		Leg_Candidate.push_back(*it);
  }

  /*
  for(auto it= Leg_Candidate.begin(), ite= Leg_Candidate.end(); it!=ite; ++it)
  {
   printf("--%f\n",it->dis_last);
  }
  */
  printf("get Leg Candidate for people:%d\n",Leg_Candidate.size());

  if(Leg_Candidate.size()!=0)
  {
      std::sort(Leg_Candidate.begin(), Leg_Candidate.end(), Leg_Candidate_Sort);//距离排序，腿的位置与预测的人的位置，距离的远近排序，越近的腿，是当前实际人腿的可能性越大！！

	  float temp_x=0;
	  float temp_y=0;
      
      int leg_num =0;

    success_num++;

	  for(auto it= Leg_Candidate.begin(), ite= Leg_Candidate.end(); it!=ite; ++it)
	  {
	    //统计有效leg的个数，最多为2，最近的两条腿，有效
        leg_num = leg_num + 1;

		temp_x = temp_x + it->avg_x;
		temp_y = temp_y + it->avg_y;
    
    printf("curve:%f\n",it->curve);
 
		if (leg_num == 2)
		  break;
	  }
	  


	  people_.x = temp_x/leg_num;
	  people_.y = temp_y/leg_num;
	  people_cluster.push_back(people_);

      //设置预测的速度
      get_predict_vel = 1;
      predict_vel_x = people_.x - last_x;
      predict_vel_y = people_.y - last_y;
      
      x_(2)=predict_vel_x;
      x_(3)=predict_vel_y;

      z_<<people_.x,people_.y;

      Kalman_Update(z_);

      if( (x_(0)-last_x<0.005)&&(x_(1)-last_y<0.002))
      {
        montionless_flag=true;
      }

      last_x = x_(0);
      last_y = x_(1);

	  Pub_People_Mark(last_x,last_y,99);
    Pub_Speed();
      //写数据到文件
      // if(outfile){
      //   printf("write to file\n");
      //   outfile<<std::setprecision(3)<<std::setfill('0')<<last_x<<'\t'<<std::setprecision(3)<<std::setfill('0')<<last_y<<'\n';

      //}
  }
  else
  {
    get_predict_vel++;
    //当前帧的没有找到people，没有得到速度，则不使用速度预测
    if(get_predict_vel==2){
	    get_predict_vel = 0;
    }
  }
  //更新加速度
  if(count>update_n)
  {
    Vel_Acc_Update(update_n);
    last_per_x =last_x;
    last_per_y =last_y;
    vel_x=predict_vel_x;
    vel_y=predict_vel_y;
    count=0;
  }
  //printf("here\n");
  count++;

  return people_cluster;
}

//目前没有用
bool Robot::follower(follower_server::follow_srv::Request  &req,follower_server::follow_srv::Response &res)
    {
     if(req.follow == "start")
     {
        start_follow = true;
        res.people_posion.push_back(people_.x);
        res.people_posion.push_back(people_.y); 
        res.laser_track_vel.push_back(vel_cmd.linear.x);
        res.laser_track_vel.push_back(vel_cmd.angular.z);

     }
     else if( req.follow == "stop")
        start_follow = false;
     else if( req.follow == "finish")
     {
        follow_finish = true;
     }
     else if(req.follow == "restart")
        {
          first_data =true;
          start_follow = true;
        }
        

      return true;
    }

/*//服务器给出激光下的跟随速度。
bool Robot::follower(follower_server::follow_srv::Request  &req,follower_server::follow_srv::Response &res)
    {
      printf("enter server!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      printf("req = %s\n",req.follow.c_str());
	    if(req.follow == "start")
	    {
		      //根据people，向客户端发布速度
		      if(get_predict_vel)
		      {
            Pub_Speed(); 
            
            res.laser_track_vel.push_back(vel_cmd.linear.x);
            res.laser_track_vel.push_back(vel_cmd.linear.y);
            res.laser_track_vel.push_back(vel_cmd.angular.z);
            
            res.people_posion.push_back(people_.x);
            res.people_posion.push_back(people_.y); 
            
            return  true;
		      }
          else
          {
            return false;
          }
		      
	    
	    }
	    if(req.follow == "kinect")
	    {
           printf("enter kinect!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	         people_.x = req.kinect_position_y;
		       people_.y = req.kinect_position_x;
	     	   Pub_Speed(); 
	         
			      
		       res.laser_track_vel.push_back(vel_cmd.linear.x);
		       res.laser_track_vel.push_back(vel_cmd.linear.y);
		       res.laser_track_vel.push_back(vel_cmd.angular.z);
    
		
		       //get_predict_vel = true;
           printf("kincet_vel_x = %f------------",vel_cmd.linear.x);
	         return true;
	    }
        return false;
    }*/

void Robot::Pub_Leg_Mark(float x, float y, int id, float time)
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

void Robot::Pub_People_Mark(float x, float y, int id, float time)
{
						visualization_msgs::Marker m;
						m.header.stamp = ros::Time::now();
						m.header.frame_id = "laser";
						m.ns = "PEOPLE";
						m.id = id;
						m.type = m.SPHERE;
						m.pose.position.x = x;
						m.pose.position.y = y;
						m.pose.position.z = 0.0;
						m.scale.x = .2;
						m.scale.y = .2;
						m.scale.z = .2;
						m.color.a = 1;
						m.color.g = 1;
						m.lifetime = ros::Duration(time);
						markers_pub_.publish(m);
}

void Robot::Pub_Text_Mark(float x, float y, int id, std::string title, float time)
{
				visualization_msgs::Marker m_;
				m_.header.stamp = ros::Time::now();
				m_.header.frame_id = "laser";
				m_.ns = "TEXT";
				m_.id = id;
				m_.type = m_.TEXT_VIEW_FACING;
				m_.pose.position.x = x;  
				m_.pose.position.y = y;  
				m_.pose.position.z = 0.0;
				m_.pose.orientation.w = 1.0;
				m_.text = title;
				m_.scale.z = 0.2;
				m_.color.a = 1;
				m_.color.r = 0.5;
				m_.color.g = 0.5;
				m_.lifetime = ros::Duration(time);
				markers_pub_.publish(m_);
}

void Robot::Pub_Speed()
{
    
    //以距离为阈值给出线速度
    float flw_dist = sqrt(last_x*last_x + last_y*last_y);
    float diff_dist = flw_dist - keep_dist;//安全距离也可以改动
    float flw_linear = diff_dist * 0.6;
    if(std::fabs(flw_linear) > 0.05)
    {
        vel_cmd.linear.x = flw_linear;
        if( vel_cmd.linear.x > max_linear_vel ) vel_cmd.linear.x = max_linear_vel;
        if( vel_cmd.linear.x < -max_linear_vel ) vel_cmd.linear.x = -max_linear_vel;
        if( vel_cmd.linear.x < 0 ) vel_cmd.linear.x *= 1.0;
    }
    else
    {
        vel_cmd.linear.x = 0;
    }

    if(flw_dist>2.8)
    {
      vel_cmd.linear.x = 0.2;
    }
    
    //给出角速度
    float d_angle = 0;
    float abs_x = std::fabs(last_x);
    if(abs_x != 0) d_angle = atan(last_y/last_x) - 0.04;
    float flw_turn = d_angle * 1.5;//角速度根据人偏移激光中心的角度而变的
    if(std::fabs(flw_turn) > 0.1)
    {
        vel_cmd.angular.z = flw_turn;
        if( vel_cmd.angular.z > max_angular_vel ) vel_cmd.angular.z = max_angular_vel;
        if( vel_cmd.angular.z < -max_angular_vel ) vel_cmd.angular.z = -max_angular_vel;
    }
    else
    {
        vel_cmd.angular.z = 0;
    }
    printf("线速度：｛%f,%f｝角速度｛%f｝\n",vel_cmd.linear.x,vel_cmd.linear.y, vel_cmd.angular.z);
    //发布速度
    speed_pub_.publish(vel_cmd);
}
// void Robot::Kalman_Predict()
// {
//   x_ = F_ * x_;
// 	MatrixXd Ft = F_.transpose();
// 	P_ = F_ * P_ * Ft + Q_ ;
// }

void Robot::Kalman_Update(const VectorXd &z) {
	VectorXd z_pred = H_laser_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
  printf("x_:%f,y:%f,k:%f\n",x_(0),y(0),K(0));
	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;
}

void Robot::Vel_Acc_Update(int &update_n)
{
  double ss_x = x_[0]-last_per_x;
  double ss_y = x_[1]-last_per_y;
  noise_ax= 4*(ss_x-vel_x*update_n)/update_n/update_n;
  noise_ay= 4*(ss_y-vel_y*update_n)/update_n/update_n; 
  predict_vel_x=ss_x/update_n;
  predict_vel_y=ss_y/update_n;

}


void Robot::Frame_Diff(const sensor_msgs::LaserScan::ConstPtr& scan){
    
  diff_count++;
  for(int i = 0;i<scan_len;i++){

    diff_data[i]=diff_data[i]+(scan_data.ranges[i]-last_scan_data.ranges[i]);
    //两帧雷达数据小于一定阈值则认定为背景点
    if(diff_data[i]<(0.005*diff_count))
    {
      scan_data.ranges[i]=std::numeric_limits<float>::quiet_NaN();
    }
  }

    printf("diff_data:%lf\n",diff_data[300]);
          last_scan_data.ranges.resize(scan_len);
          last_scan_data.ranges=scan->ranges;
          last_scan_data.header.frame_id = scan->header.frame_id;
          last_scan_data.angle_min = scan->angle_min;
          last_scan_data.angle_max = scan->angle_max;
          last_scan_data.angle_increment = scan->angle_increment;
          last_scan_data.intensities=scan->intensities;
          last_scan_data.scan_time = scan->scan_time;
            //            printf("angle_increment:%d\n",scan_data.angle_increment);
          last_scan_data.time_increment = scan->time_increment;
        last_scan_data.header.stamp = scan->header.stamp;
        last_scan_data.range_min= scan->range_min;
        last_scan_data.range_max= scan->range_max;
      scan_pub.publish(scan_data);
}
