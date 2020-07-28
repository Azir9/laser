#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <nav_msgs/GetPlan.h>

#include <vector>
#include "std_msgs/String.h"

#include<stdio.h>
#include<algorithm>
#include<iostream>

#include <sensor_msgs/LaserScan.h>

#include <angles/angles.h>
#include "openni_tracker_test/skeleton.h"
#include "openni_tracker_test/SetFollowState.h"
#include "xf_voice/voice_srv.h"

#include <std_srvs/Empty.h>
#include "follower_server/follow_srv.h"
#include "serial/serial.h"
#include <ros/package.h>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"




#include <chrono>
//#include <cstdlib>
//#include <math.h>
//#include <mutex>
//#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

  typedef struct SHOP_gy
  {
      std::string good_name;
      tf::StampedTransform transform;
  }SHOP_gy;

class Shopping
{
    public:
      //记录任务用
          ros::ServiceClient clear_costmaps_client;//清理代价地图

        tf::TransformListener listener;
        tf::StampedTransform transform;

        MoveBaseClient move_base_client;
        
        ros::ServiceClient follow_client;
        ros::ServiceClient tts_client;

        ros::Publisher test_pub;
        ros::Publisher speed_pub_;
        ros::Subscriber sub_kinect;
        ros::Subscriber serial_sub;

        ros::Subscriber darknet_sub;
        ros::Publisher serial_pub;


      //  ros::Subscriber sub_laser;

        geometry_msgs::Twist vel_cmd_move;
        ros::Rate loop_rate;

        chrono::steady_clock::time_point last_time;
        
        std::vector<SHOP_gy> V_shop_list;
        std::vector<SHOP_gy> V_shop_navigation;
        std::vector<darknet_ros_msgs::BoundingBox> Box_vec;
        ros::Rate loop_rate_5;

  
  geometry_msgs::Quaternion quat;


  tf::StampedTransform transform_cash;//收银台位置
  float nearst_lser_distance_last[2];
  float people_posion_kinect[2];// x,y不需要z
  float people_posion_laser[2];//x,y 

  bool voice_rec_state=false;
  bool voice_start_follow;
  int voice_stop_follow;
  int landmark;
  int tracking_step;
  bool start_follow;
  bool stop_follow;
  float Scan_Data[512];
  int kinect_index;
  string shop_name[4];
  int shop_num;
  bool navigation_to_goods;
  int voice_count[4];
  //int scan_len;
  int good_num=0;

  double min_y_= -0.20; /**< The minimum y position of the points in the box. */
  double max_y_= 0.20; /**< The maximum y position of the points in the box. */
  double min_x_= 0.8; /**< The minimum x position of the points in the box. */
  double max_x_= 1.0; /**< The maximum x position of the points in the box. */
  double goal_x_= 0.9; /**< The distance away from the robot to hold the centroid */
  double x_scale_= 0.5; /**< The scaling factor for translational robot vel_cmd_move */
  double y_scale_ = 1.2; /**< The scaling factor for rotational robot vel_cmd_move */
  double vel_angle=0,vel_linear=0;
  
    darknet_ros_msgs::BoundingBoxes darknet_msg;

  follower_server::follow_srv follow_srv;
  openni_tracker_test::skeleton skeleton_information;

  //float obsc_vel[2];
  //float d_kinect_position[2];
  //float kinect_position_last[2];
  float kinect_position[2];
  //float laser_position_last[2];
  xf_voice::voice_srv tts_srv;
  std::string voice_action_temp = "";
  std::string voice_squ = "";  
  std::vector<std::string> vWord; 
    int catch_flag = -1;//抓取标志位
 std::string result = "";
std_msgs::String catch_order;
  std_srvs::Empty ccm;//清理代价地图



  ros::NodeHandle nh;

  Shopping():move_base_client("/move_base", true),start_follow(false),
  stop_follow(false),landmark(1),loop_rate(20),loop_rate_5(5),tracking_step(0),
  kinect_index(-1),voice_start_follow(false),voice_stop_follow(0),navigation_to_goods(false),shop_num(0)
  {
    sub_kinect = nh.subscribe("/kinect_skeleton",1, &Shopping::kinect_callback, this); //订阅kinectkinect_order话题
    //sub_laser = nh.subscribe("/chatter", 1, &Shopping::laser_callback, this); 
    tts_client = nh.serviceClient<xf_voice::voice_srv>("voice");
    //sub_laser = nh.subscribe("/scan", 1, &Shopping::laser_callback, this);  //订阅激光数据
    clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    speed_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20); //发布消息
    follow_client = nh.serviceClient<follower_server::follow_srv>("follow");
    darknet_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Shopping::darknet_callback, this);

    serial_pub = nh.advertise<std_msgs::String>("write", 1);
    serial_sub = nh.subscribe("read", 1, &Shopping::catch_callback, this);


  }


   int Split(const std::string &s, const std::string &splitchar, std::vector <std::string> &vec)
  {
	printf("---------------split names--------------------------------\n");
    std::string stmp = "";
	std::cout<<"voice_temp:-----------\n"<< s <<std::endl;
	std::string::size_type pos = 0, prev_pos = 0;
	int num_str = 0;
    vec.clear();
	while ((pos = s.find_first_of(splitchar, pos)) != std::string::npos)
	{
		stmp = s.substr(prev_pos, pos - prev_pos);   
		vec.push_back(stmp);
		prev_pos = ++pos;
 		num_str++;
	}
	return num_str;
 }


/* void cEulerAngle_to_Quat(double _yaw) 
{
    double cos_hroll = cos(0);
    double sin_hroll = sin(0);
    double cos_hpitch = cos(0);
    double sin_hpitch = sin(0);
    double cos_hyaw = cos(_yaw / 2.0);
    double sin_hyaw = sin(_yaw / 2.0);
    quat.w = cos_hroll*cos_hpitch*cos_hyaw + sin_hroll*sin_hpitch*sin_hyaw;
    quat.x = cos_hroll*sin_hpitch*sin_hyaw - sin_hroll*cos_hpitch*cos_hyaw;
    quat.y = -sin_hroll*cos_hpitch*sin_hyaw - cos_hroll*sin_hpitch*cos_hyaw;
    quat.z = -cos_hroll*cos_hpitch*sin_hyaw + sin_hroll*sin_hpitch*cos_hyaw;
}
double QuaternionToEulerAngles(double qw, double qx, double qy, double qz)
{
    double roll, yaw, pitch;
    yaw =  asinf(2.f * (qw*qx - qy*qz)); //Y
 
    cout << "roll = " << roll << endl;
    cout << "yaw = " << yaw << endl;
    cout << "pitch = " << pitch << endl;
    return yaw;
}*/

    void GetRobotPosition()
  {
    try
    {
        //ROS_INFO("Attempting to read pose...");
        listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
        ROS_INFO("当前机器人位置 x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
        //myfile << transform.getOrigin().x() << "," << transform.getOrigin().y() << "\n";
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Nope! %s", ex.what());
    } 

  }


  void kinect_callback(const openni_tracker_test::skeleton::ConstPtr& msg)
  {
        printf("debug----001");
        skeleton_information.skeleton_num = msg->position_x.size();
	    skeleton_information.position_x = msg->position_x;
	    skeleton_information.position_y = msg->position_y;
        skeleton_information.order = msg->order;

        //初始校准过程
        follow_srv.request.follow = "start";
        
        string skeleton_order = " ";
        
        skeleton_order = skeleton_information.order;

        
        printf("shop＿num---------%d-----0\n", V_shop_list.size());
        printf("skeleton_order-----%s\n",skeleton_order.c_str());
        if (skeleton_order == "stop follow" && V_shop_list.size() >= 2)//停止状态且货物已经记录完毕，此时应该在收银台
        {
            //识别记录
            GetRobotPosition();
            transform_cash = transform;         
            printf("V_shop_list--------------0\n");

            for(int i=0;i<V_shop_list.size();i++)
            {
              printf("---------%s-----0\n", V_shop_list[i].good_name.c_str());
            }

            //开始语音交互
            voice_rec_state = false;
            int word_number=0;//语音识别得到的单词数
            V_shop_navigation.clear();
            for(int i=0;i<3;i++)
            {	
              if(i<3 && !voice_rec_state && V_shop_list.size() >= 4) //如果说名字没有超过三次且mynameis没有识别成功  
              {
                  tts_srv.request.cmd = "tts";
                  tts_srv.request.txt = "please tell the task";
                  bool v_recognize  = false;
                            
                  if(tts_client.call(tts_srv))//语音合成成功，开始进行asr
                  {
                    printf("start asr!\n");
                    sleep(1);
                  }
                  tts_srv.request.cmd = "asr";
                  v_recognize = tts_client.call(tts_srv);
                  //roscpp.wait_for_service("tts_client");
        
                  if(v_recognize)//语音识别识别成功与否					
                  {
                    //voice_action_temp = tts_srv.response.action;
                    voice_squ = tts_srv.response.squ;
                    for(int j=0;j<3;j++)
                    {
                      tts_srv.request.cmd = "tts";
                      tts_srv.request.txt = "did you say"+voice_squ;
                      tts_client.call(tts_srv); 
                      sleep(3);
                      tts_srv.request.cmd = "asr";
                      if(tts_client.call(tts_srv))
                      {  
                        std::string voice_action_temp2=tts_srv.response.squ;
                        std::cout<<"voice_action_temp2:--------------"<<voice_action_temp2<<std::endl;
                        if (voice_action_temp2=="right \n")
                        {
                          j=3;
                            voice_rec_state = true;
                          //voice_rec_state = true;//voice_rec geted!
                          //TODO::将名字截取出来函数
                            word_number = Split(voice_squ," ",vWord);
                          ROS_INFO("[%d]word_number:%d---------------\n",i,word_number);
                          ROS_INFO("vword_size:%d---------------\n",vWord.size());
                          ROS_INFO("voice_rec_state:%d---------------\n",voice_rec_state);

                        }
                        else
                        { 
                            i--;
                                j=3;
                          ROS_INFO("repeat1(you are wrong): -----------please tell me your name\n");
                          sleep(1);
                        }
                      }
                      else
                      {
                        ROS_INFO("repeat2(dont rec voice2):-----------did you say\n");
                        sleep(1);
                      }
                    } 	
                  }
                  else
                  {
                              ROS_INFO("repeat3(dont rec voice1):--------please tell me your task\n");
                              sleep(1);
                  }
              }
          }//for end

            if(vWord.size()>1 && voice_rec_state)//这里判断了传回的话是否为空的．
            {
                    navigation_to_goods=true;
                    //TODO::get V_shop_navigation
                    V_shop_navigation.clear();
                    std::cout<<"get V_shop_navigation"<<std::endl;
                    for(int i=0;i<vWord.size();i++)
                    {
                        string temp_a=vWord[i];
                        for(int j=0;j<V_shop_list.size();j++)
                        {
                            if(temp_a==V_shop_list[j].good_name)
                            {
                                SHOP_gy good_;
                                good_.good_name = V_shop_list[j].good_name;
                                good_.transform = V_shop_list[j].transform;
                                
                                V_shop_navigation.push_back(good_);
                            }
                        }
                    }
                    if(V_shop_navigation.size()==3)
                    {
                       std::cout<<"get V_shop_navigation---end"<<std::endl;
                    }
                    else
                    {
                        voice_rec_state = 0;
                        tts_srv.request.cmd = "tts";
                      tts_srv.request.txt = "please say again";
                      tts_client.call(tts_srv); 
                        std::cout<<" not get name：---------------"<<std::endl;

                    }
            }
            else
            {
            printf("－－－shouyintai－－－past name－－－－－－－－－－－－\n");
            std::cout<<" not get name：---------------"<< shop_name[shop_num] << std::endl;
            sleep(1);									
            voice_rec_state=false;
            }

        }//end收银台
        else if (skeleton_order == "turn right"|| skeleton_order == "turn left" && V_shop_list.size() < 3)
        {
            //如果识别到有货物需要停下进行语音识别
            //首先停下
            vel_cmd_move.angular.z=0;
            vel_cmd_move.linear.x=0;
            speed_pub_.publish(vel_cmd_move);              

            string fangxiang = skeleton_order;

             //1--语音识别，记录名字 并且shop_num++
                //开始语音交互，获得名字
                vWord.clear();   
                voice_rec_state = false;
                int word_number=0;//语音识别得到的单词数
                for(int i=0;i<3;i++)
                {	
                    if(i<3&&!voice_rec_state) //如果说名字没有超过三次且mynameis没有识别成功  
                    {
                        tts_srv.request.cmd = "tts";
                        tts_srv.request.txt = "please tell the good name";
                        bool v_recognize  = false;
                        if(tts_client.call(tts_srv))//语音合成成功，开始进行asr
                        {
                                printf("start asr!\n");
                                sleep(2);
                        }
                        tts_srv.request.cmd = "asr";
                        v_recognize = tts_client.call(tts_srv);
                                //roscpp.wait_for_service("tts_client");
    
                        if(v_recognize)//语音识别识别成功与否					
                        {
                                //voice_action_temp = tts_srv.response.action;
                                voice_squ = tts_srv.response.squ;
                                for(int j=0;j<3;j++)
                                {
                                    tts_srv.request.cmd = "tts";
                                    tts_srv.request.txt = "did you say"+voice_squ;
                                    tts_client.call(tts_srv); 
                                    sleep(2);
                                    tts_srv.request.cmd = "asr";
                                    if(tts_client.call(tts_srv))
                                    {  
                                        std::string voice_action_temp2=tts_srv.response.squ;
                                        std::cout<<"voice_action_temp2:--------------"<<voice_action_temp2<<"-------------"<<std::endl;
                                        if (voice_action_temp2=="right \n")
                                        {
                                            j=3;
                                            voice_rec_state = true;//voice_rec geted!
                                            //TODO::将名字截取出来函数
                                            word_number = Split(voice_squ," ",vWord);
                                            ROS_INFO("word_number:%d---------------\n",word_number);
                                        }
                                        else
                                        {  
                                        j=3;
                                        ROS_INFO("repeat1(you are wrong): -----------please tell me the good name\n");
                                        sleep(1);
                                        }
                                    }
                                    else
                                    {
                                    ROS_INFO("repeat2(dont rec voice2):-----------did you say\n");
                                    sleep(1);
                                    }
                                } 	
                        }
                        else
                        {
                            ROS_INFO("repeat3(dont rec voice1):--------please tell me your name\n");
                            sleep(1);
                        }
                    }
                }//for end 语音识别

                if(vWord.size()>1 && voice_rec_state)//这里判断了传回的话是否为空的．
                {
                    //添加货品名字
                    std::cout<<"vWord[vWord.size()-1]:---------"<<vWord[vWord.size()-1]<<std::endl;
                    
                    //shop_list[shop_num].good_name=vWord[vWord.size()-1]; 
                    //shop_list[shop_num].num=shop_num;
                    //shop_num++;

                    tts_srv.request.cmd = "tts";
                    tts_srv.request.txt = "Get the name"+vWord[vWord.size()-1];
                    tts_client.call(tts_srv);     			
                    
                }
                else
                {
                    printf("－－－－－－past name－－－－－－－－－－－－\n");
                    std::cout<<" 394not get name：---------------"<< shop_name[shop_num] << std::endl;
                    
                    sleep(1);									
                    voice_rec_state=false;
                }


            //TODO２:转体记录位置，再转回来
            if(voice_rec_state)
            {
                tts_srv.request.cmd = "tts";//"stop","restart"
                if (fangxiang == "turn left")
                {
                    tts_srv.request.txt = "向左";
                    tts_client.call(tts_srv);

                    //记录该物品位置
                    GetRobotPosition();

                    //vector
                    SHOP_gy good_;
                    good_.good_name = vWord[vWord.size()-1];
                    //good_.num = 0;
                    
                    double yaw = tf::getYaw(transform.getRotation());
                    printf("yaw:%f",yaw);
                    
                    transform.setRotation(tf::createQuaternionFromYaw(yaw+1.507));

                    good_.transform = transform; // good transform需转90度
                    V_shop_list.push_back(good_);
                    good_num++;
                    fangxiang = "";

                }
                else if(fangxiang == "turn right")//向右
                {
                    tts_srv.request.txt = "向右";
                    tts_client.call(tts_srv);

                    //记录该物品位置
                    GetRobotPosition();

                    SHOP_gy good_;
                    good_.good_name = vWord[vWord.size()-1];
                    //good_.num = 0;

                    //transform 方向转90度

                    double yaw = tf::getYaw(transform.getRotation());
                    printf("yaw:%f",yaw);
                    
                    transform.setRotation(tf::createQuaternionFromYaw(yaw-1.507));


                    good_.transform = transform;
                    V_shop_list.push_back(good_);


                } 
            }
            //TODO::Why?????
            skeleton_order = "start follow";
        }//end elseif

        //跟踪状态
        if(!navigation_to_goods)
        {
            switch (tracking_step)
            {  
            case 0:
                    {
                        //初始校准阶段
                        //请求跟随服务获取激光数据下人的位置
                        if(follow_client.call(follow_srv))
                        {
                            printf("111111\n");
                            
                            people_posion_laser[0] = follow_srv.response.people_posion[0]; 
                            people_posion_laser[1] = follow_srv.response.people_posion[1];
                              
                            people_posion_laser[0] = people_posion_laser[0]+0.10; //激光坐标系与kinect坐标系在xy平面对齐   

                            //激光数据找到人
                            printf("teste\n");
                            if ( read_kinect_position())
                            {

                                voice_count[2] = 0;
                                voice_count[1] = 0;
                                voice_count[3] = 0;

                                if( kinect_index>=0)
                                {
                                        tts_srv.request.cmd = "tts";//"stop","restart"
                                        tts_srv.request.txt = "校准完成，可以启动跟随"; 
                                        if(voice_count[0]<1)
                                        {
                                        tts_client.call(tts_srv);
                                        sleep(2);//睡觉函数参数是否正确，在此处使用是否合适。
                                        voice_count[0]++;
                                        }
                                        //转入第二阶段
                                        tracking_step = 1;
                                        voice_start_follow = true;
                                }
                                else
                                {
                                        //有人但是与激光数据不匹配
                                    people_posion_kinect[0] = people_posion_laser[0];
                                    people_posion_kinect[1] = people_posion_laser[1];
                                    voice_count[3] = 0;
                                    voice_count[2] = 0;
                                        
                                    voice_count[0] = 0;
                                    tts_srv.request.cmd = "tts";//"stop","restart"
                                    tts_srv.request.txt = "匹配失败，请调整"; 
                                    if(voice_count[1]<3)
                                    {
                                        tts_client.call(tts_srv);
                                        sleep(1);				
                                        voice_count[1]++;
                                        }//tts_client.call(tts_srv); 
                                    //睡觉函数参数是否正确。睡觉函数参数是否正确，在此处使用是否合适
                                    
                                }

                            }

                                else
                                {    //没有人
                                    people_posion_kinect[0] = people_posion_laser[0];
                                    people_posion_kinect[1] = people_posion_laser[1];

                                  voice_count[3] = 0;
          
                                  voice_count[1] = 0;
                                  voice_count[0] = 0;
                                  tts_srv.request.cmd = "tts";//"stop","restart"
                                  tts_srv.request.txt = "请移动位置，以获取对象"; 
                                  printf("please move\n");                                
                                  if(voice_count[2]<3)
                                  {
                                      tts_client.call(tts_srv);
                                      sleep(1);
                                      voice_count[2]++;
                                  }
                                }

                            }
                        else
                        {
                            people_posion_laser[0] = 0.0;
                            people_posion_laser[1] = 0.0;
                            //激光没找到人
                            printf("The scan can not find a person\n");
                            voice_count[2] = 0;
                            voice_count[1] = 0;
                            voice_count[0] = 0;
                            tts_srv.request.cmd = "tts";//"stop","restart"
                            tts_srv.request.txt = "激光未获取目标";
                            if(voice_count[3]<3)
                            {
                                tts_client.call(tts_srv);
                                //printf("the prog enter here -------22222222222\n");
                                sleep(1);
                                voice_count[3]++;
                            }
                        }
                   
                        break;
                    }
            case 1:{
                        //跟随阶段
                        if(read_kinect_position() >=0)
                        {
                            //激光数据找到人
                            //获得激光数据下人的位置
                            people_posion_laser[0] = follow_srv.response.people_posion[0]; 
                            people_posion_laser[1] = follow_srv.response.people_posion[1];
                                
                            people_posion_laser[0] = people_posion_laser[0]+0.10; //激光坐标系与kinect坐标系在xy平面对齐   
                            
                            //read_kinect_position();
                            // 激光能找到人，直接已激光跟随速度驱动机器人运动
                            printf("test2");
                            // vel_cmd_move.linear.x = follow_srv.response.laser_track_vel[0];
                            // vel_cmd_move.angular.z = follow_srv.response.laser_track_vel[2];
                            // vel_cmd_move.linear.y = 0;
                            people_posion_kinect[0] = people_posion_laser[0];
                            people_posion_kinect[1] = people_posion_laser[1];
                        }
                        else
                        {
                            //激光找不到人，需要利用Kinect进行跟随，同时利用激光避障
                            //此处假设在此阶段不会出现激光找不到人的情况，因此代码不写如有需要可根据下一阶段代码修改添加
                            //置速度为0，机器人停止运动
                            follow_srv.request.follow = "stop";
                            follow_client.call(follow_srv);
                            kinect_index  = -3;
                        }

                        //std::cout<<"---"<<msg->data<<std::endl; 此处为骨架状态，判断启动命令
                        string order ="";
                        //需要查找动作发出者是否为标记对象
                        if (kinect_index>=0)
                        {
                            //查找kinect_index的骨架信息动作 
                            printf("test3\n"); 
                            printf("kinect_index = %d\n",kinect_index);
                            //printf("skeleton_information.order.size() = %d\n",skeleton_information.order.size());
                            order = skeleton_information.order;
                        }
                        else if (kinect_index == -1)
                        {
                            //有人但是与激光数据不匹配，以距离激光测量位置最近的人作为骨架判断对象
                            printf("test5\n");
                            printf("skeleton_information.num = %d\n",skeleton_information.skeleton_num);
                            order = skeleton_information.order;
                            //计算预测位置
                        }

                        else if (kinect_index == -2)
                        {
                            //骨架没有数据
                            ROS_INFO("丢失骨架数据\n");
                        }
                        else if (kinect_index == -3)
                        {
                            //激光找不到人，距离上一帧激光找到位置最近的人作为判断对像
                                order = skeleton_information.order;
                            
                        }



                        if(order == "start follow")
                        {
                            start_follow = true;
                            stop_follow = false;
                            if (voice_start_follow)
                            {
                                tts_srv.request.cmd = "tts";//"stop","restart"
                                tts_srv.request.txt = "开始跟随";
                                voice_start_follow = false;
                                tts_client.call(tts_srv);
                                follow_srv.request.follow = "start";
                                while(!follow_client.call(follow_srv))
                                {
                                    printf("failed to call follow server\n");
                                }
                            }
                            
                            printf("start follow\n");                      
                        }

                        if(order == "stop follow")
                        {
                            start_follow = true;
                            stop_follow = false;

                                follow_srv.request.follow = "start";
                                while(!follow_client.call(follow_srv))
                                {
                                    printf("failed to call follow server\n");
                                }

                                
                        }

                        break;
                    }

            default:
                    ;
            }
        }
  }
    //参数返回说明：true 说明kinect有人 匹配成功 标签参数为找到的第几个人
  //                                 匹配失败 标签参数为-1，
  //             false 说明没有人标签参数为-2

  bool read_kinect_position()
  {
       if (skeleton_information.skeleton_num>0)
       {
           //kinect 找人
           float error_dis_LandK = 0;
           float kinect_posion[2];
           for( int i = 0;i<skeleton_information.skeleton_num;i++)
           {
               kinect_posion[0] = skeleton_information.position_x[i];
               kinect_posion[1] = skeleton_information.position_y[i];
               //float height  = skeleton_information.position[i*3+2]/1000.0;
               printf("kinect.x = %f, kinect.y= %f \n" ,kinect_posion[0],kinect_posion[1] );
                printf("laser.x = %f, laser.y= %f \n" ,people_posion_laser[0],people_posion_laser[1] );
               
               error_dis_LandK = sqrt((people_posion_laser[0]-kinect_posion[0])*(people_posion_laser[0]-kinect_posion[0])+(people_posion_laser[1]-kinect_posion[1])*(people_posion_laser[1]-kinect_posion[1]));
               printf("error_dis_LandK= %f \n" ,error_dis_LandK );
               //if (error_dis_LandK<=0.8 && height>1.4 && height<1.9) //两传感器测的人员距离差 
               if (error_dis_LandK<=0.8) //两传感器测的人员距离差 
               {
                   
                   people_posion_kinect[0] = kinect_posion[0];
                   people_posion_kinect[1] = kinect_posion[1];
                   kinect_index = i;
                   printf("kinect_index = %d\n",kinect_index );
                   break;
               }
               else
               {
                   kinect_index = -1;
               }
               
           }
           
           return true;
       }
       else
       {
           kinect_index = -2;
           return false;
       }
  }

/*  bool kinect_speed(float hunman_pos_x, float hunman_pos_y)
  {
      	if(hunman_pos_x != 0 && hunman_pos_x<2.2 && hunman_pos_y < 1.5){
			if(hunman_pos_x>(max_x_) || hunman_pos_x <(min_x_))
			{
				vel_linear= (hunman_pos_x - goal_x_)*x_scale_;
					printf("vel_linear: %f\n", vel_linear);

			}
			if(hunman_pos_y>(max_y_) || hunman_pos_y <(min_y_))
			{
				vel_angle= (hunman_pos_y)*y_scale_;
				printf("vel_angle: %f\n", vel_angle);

			}
            return true;
		} 
        else{
            return false;
        }
  }
*/

        void darknet_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
        {
            ROS_INFO("enter the darknet_callback!");

            darknet_ros_msgs::BoundingBox Box_temp;
            Box_vec.clear();

            darknet_msg.bounding_boxes = msg->bounding_boxes;
            ROS_INFO("darknet_msg.bounding_boxes.size:%ld", darknet_msg.bounding_boxes.size());


            for(int i=0; i<darknet_msg.bounding_boxes.size(); i++)
            {
                Box_temp = darknet_msg.bounding_boxes[i];
                Box_vec.push_back(Box_temp);
            }

            for(int j=0; j<Box_vec.size(); j++)
            {
                printf("Box_vec[%d].Class:%s\n", j, Box_vec[j].Class.c_str());
                printf("Box_vec[%d].probability:%f\n", j, Box_vec[j].probability);
                printf("Box_vec[%d].xmin:%ld\n", j, Box_vec[j].xmin);
                printf("Box_vec[%d].xmax:%ld\n", j, Box_vec[j].xmax);
                //printf("Box_vec[%d].ymin:%ld\n", j, Box_vec[j].ymin);
                //printf("Box_vec[%d].ymax:%ld\n", j, Box_vec[j].ymax);
            }

        }
        bool catch_return()
        {
            if(result[0] == '.' && result[1] == 'Y')
            {
                catch_flag = 1;
                if(MoveForward(-1.2))//后退
                {
                    sleep(10);
                    catch_order.data = ".y\n\r";
                    serial_pub.publish(catch_order);
                    printf("--------catching succeed!-------\n");

                }
                return true;
            }
            else if(result[0] == '.' && result[1] == 'N')
            {
                catch_flag = 0;
                MoveForward(-1.2);//后退
                sleep(10);
                catch_order.data = ".n\n\r";
                serial_pub.publish(catch_order);
                printf("--------catching failed!-------\n");


                return true;
            }
            
            return false;
        }
        void catch_callback(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO_STREAM("get the serial call back\n" << msg->data);
            result = msg->data;

        }

        bool TurnCatch(std::string target_good)
        {
            darknet_ros_msgs::BoundingBox object_box;
            bool object_flag; //视野内有无目标

            bool go_for_catch = 0;
            std::string goal_class;
            float object_center;
            int search_time = 0;
            int loop_time = 0;


        goal_class = target_good;

            while(!go_for_catch)
            {
                object_flag = 0;
                ros::spinOnce();
                loop_rate.sleep();
                sleep(2);

                for(int i=0; i<Box_vec.size(); i++) //目前是没有一样的物品，人脸识别项目版本
                {
                    ROS_INFO("Box_vec[%d].Class:%s", i, Box_vec[i].Class.c_str());
                    if(goal_class == Box_vec[i].Class)
                    {
                        object_box = Box_vec[i];
                        object_center = (object_box.xmax+object_box.xmin)/2.0;
                        object_flag = 1;
                    }
                }

                if(object_flag)
                {

                    if(object_center < 290)
                    {
                        
                        
                        TurnLeft(8, 1.3);
                        sleep(1.0);
                        Stoprobot();
                    }
                    else if(object_center > 350)
                    {
                        
                        
                        TurnRight(8, 1.3);
                        sleep(1.0);
                        Stoprobot();
                    }
                    else
                    {
                        go_for_catch = 1;
                    }
                }
                else
                {
                    ROS_INFO("There is no target here!");
                    search_time++;
                    if(search_time <= 3)
                    {
                        //左转一定角度
                        TurnLeft(30, 1.5);
                        Stoprobot();
                    }
                    if(search_time <= 9 && search_time > 3)
                    {
                        // 右转一定角度
                        TurnRight(30, 1.5);
                        Stoprobot();
                    }
                    else if(search_time >9){
                        search_time = 0;
                        break;
                    }
                }
                //不变时没三次更新一次
                loop_time++;
                if(loop_time>3)
                {
                     Box_vec.clear();
                }
          }
            sleep(2);
            return go_for_catch;
        }


        void Stoprobot()
        {
            vel_cmd_move.linear.x = 0.0;
            vel_cmd_move.angular.z = 0.0;
            speed_pub_.publish(vel_cmd_move);  

        }
        void TurnLeft(int time, float rate)
        {
            for(int i=0; i<time; i++)
        {
            vel_cmd_move.linear.x = 0.0;
            vel_cmd_move.angular.z = 0.1 * rate; 
            speed_pub_.publish(vel_cmd_move); 
            loop_rate.sleep();
        }
        }

        void TurnRight(int time, float rate)
        {
            for(int i=0; i<time; i++)
        {
            vel_cmd_move.linear.x = 0.0;
            vel_cmd_move.angular.z = -0.1 * rate; 
            speed_pub_.publish(vel_cmd_move); 
            loop_rate.sleep();
        }
        }

        bool MoveForward(float qh)
        {
        for(int i=0; i<8; i++)
        {
            vel_cmd_move.linear.x = 0.15 * qh;
            vel_cmd_move.angular.z = 0.0; 
            speed_pub_.publish(vel_cmd_move); 
            loop_rate.sleep();
        }
        return 1;
        }
};


int main(int argc, char** argv) {
ros::init(argc, argv, "shopping");
//ros::Time::init();

Shopping Shopping;
//Shopping.last_time = std::chrono::steady_clock::now();

ROS_INFO("Waiting for the move_base action server");
Shopping.move_base_client.waitForServer(ros::Duration(60));
ROS_INFO("Connected to move base server");

  cout <<"start Shopping---------\n"<<endl;
std_msgs::String q_order;

while (ros::ok())
{
    if(Shopping.navigation_to_goods)
    {
            int bb = 0;
            bool q_flag = 0;
            Shopping.tts_srv.request.cmd = "tts";
            Shopping.tts_srv.request.txt = "Get the task";
            std::cout<<"----shoplist----"<<std::endl;
            for(int i=0;i<Shopping.V_shop_navigation.size();i++)
            {
                std::cout<<"--"<<Shopping.V_shop_navigation[i].good_name<<std::endl;
                Shopping.tts_srv.request.txt = Shopping.tts_srv.request.txt + Shopping.V_shop_navigation[i].good_name;
            }
            Shopping.tts_client.call(Shopping.tts_srv);
            std::cout<<"go the goods"<<std::endl;
            Shopping.clear_costmaps_client.call(Shopping.ccm);
            bb = system("gnome-terminal \"usb_cam\" -x bash -c \" rosrun usb_cam usb_cam_node;\"");
            bb = system("gnome-terminal \"darknet_ros\" -x bash -c \" rosrun darknet_ros darknet_ros;\"");
                                 sleep(6);
          for (int root_i=0;root_i<Shopping.V_shop_navigation.size();root_i++){
                //shop_list.clear;

                move_base_msgs::MoveBaseGoal goal_task;
                goal_task.target_pose.header.frame_id = "map";
                goal_task.target_pose.header.stamp = ros::Time::now();
                std::cout<<"set the goal of the goods"<<std::endl;
                goal_task.target_pose.pose.position.x = Shopping.V_shop_navigation[root_i].transform.getOrigin().x();//需要更改
                goal_task.target_pose.pose.position.y = Shopping.V_shop_navigation[root_i].transform.getOrigin().y();//需要更改
                tf::Quaternion tfq =Shopping.V_shop_navigation[root_i].transform.getRotation(); 
                std::cout<<"set the goal of the goods----end"<<std::endl;        
                quaternionTFToMsg(tfq, goal_task.target_pose.pose.orientation);
                Shopping.tts_srv.request.cmd = "tts";
                Shopping.tts_srv.request.txt = "go to the task"+Shopping.V_shop_navigation[root_i].good_name;
                Shopping.tts_client.call(Shopping.tts_srv);         
                ROS_INFO("Go for task");
                Shopping.move_base_client.sendGoal(goal_task);
                Shopping.move_base_client.waitForResult();
                if (Shopping.move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("You have reached the goal!");
                else
                ROS_INFO("The base failed for some reason");    



                if(Shopping.move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    Shopping.tts_srv.request.cmd = "tts";
                    Shopping.tts_srv.request.txt = "reach the task" + Shopping.V_shop_navigation[root_i].good_name;
                    Shopping.tts_client.call(Shopping.tts_srv);
                    ROS_INFO("Succeeded goal !!!");

                                            //如果识别到物体且在画面中间
                        if(Shopping.TurnCatch(Shopping.V_shop_navigation[root_i].good_name))
                        {
                            printf("start catching\n");
                            Shopping.MoveForward(1);
                            while(1)
                            {
                                printf("1111111\n");
                                if (Shopping.result[0] == '.' && Shopping.result[1] == 'Q') 
                                {
                                    q_flag = 1;//发送标志位关闭
                                    ROS_INFO("%s\n\r", Shopping.result.c_str());
                                    break;
                                }
                                if(!q_flag)
                                {
                                 q_order.data = ".q\r\n";
                                 Shopping.serial_pub.publish(q_order);
                                //ROS_INFO("%s\n\r", Shopping.result.c_str());                                    
                                }
                                else{
                                    break;
                                }
                                ros::spinOnce();
                                Shopping.loop_rate.sleep();
                            }
                            printf("222222\n");
                            Shopping.result = "";
                            while(Shopping.catch_flag < 0 ) 
                            {   
                                Shopping.catch_return();
                                printf("wait for Shopping result!\n");
                                ros::spinOnce();
                                Shopping.loop_rate.sleep();

                            }

                       }
                       else
                       {
                           //没有识别也要向前，保证50cm内
                            Shopping.MoveForward(1);
                            sleep(3);
                            Shopping.MoveForward(-1);
  
                       }

                    
                }
                else
                {
                    
                    Shopping.clear_costmaps_client.call(Shopping.ccm);
                    ROS_INFO("Once Again");
                    Shopping.move_base_client.sendGoal(goal_task);
                    Shopping.move_base_client.waitForResult();	
                    if(Shopping.move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    
                        Shopping.tts_srv.request.cmd = "tts";
                        Shopping.tts_srv.request.txt = "reach the task start Shopping" + Shopping.V_shop_navigation[root_i].good_name;
                        Shopping.tts_client.call(Shopping.tts_srv);
                    ROS_INFO("Succeeded goal !!!");
                    //Shopping

                        if(Shopping.TurnCatch(Shopping.V_shop_navigation[root_i].good_name))
                         {
                            printf("start catching\n");
                            Shopping.MoveForward(1);
                            while(1)
                            {
                                printf("1111111\n");
                                if (Shopping.result[0] == '.' && Shopping.result[1] == 'Q') 
                                {
                                    q_flag = 1;//发送标志位关闭
                                    ROS_INFO("%s\n\r", Shopping.result.c_str());
                                    break;
                                }
                                if(!q_flag)
                                {
                                 q_order.data = ".q\r\n";
                                 Shopping.serial_pub.publish(q_order);
                                //ROS_INFO("%s\n\r", Shopping.result.c_str());                                    
                                }
                                else{
                                    break;
                                }
                                ros::spinOnce();
                                Shopping.loop_rate.sleep();
                            }
                            printf("222222\n");
                            Shopping.result = "";
                            while(Shopping.catch_flag < 0 ) 
                            {   
                                Shopping.catch_return();
                                printf("wait for Shopping result!\n");
                                ros::spinOnce();
                                Shopping.loop_rate.sleep();

                            }

                        }
                    }
                }

            }//end for
                    bb = system("gnome-terminal \"darknet_shutdown\" -x bash -c \" pkill -9 darknet_ros;\"");
                    bb = system("gnome-terminal \"usb_cam_shutdown\" -x bash -c \" pkill -9 usb_cam_node;\"");
                            sleep(8);//留时间关darknet
                        //返回收银台
                move_base_msgs::MoveBaseGoal goal_cash;
                goal_cash.target_pose.header.frame_id = "map";
                goal_cash.target_pose.header.stamp = ros::Time::now();
                goal_cash.target_pose.pose.position.x = Shopping.transform_cash.getOrigin().x();//需要更改
                goal_cash.target_pose.pose.position.y = Shopping.transform_cash.getOrigin().y();//需要更改
                tf::Quaternion  tfq = Shopping.transform_cash.getRotation();         
                quaternionTFToMsg(tfq, goal_cash.target_pose.pose.orientation);
                Shopping.tts_srv.request.cmd = "tts";
                Shopping.tts_srv.request.txt = "go to the cash";
                Shopping.tts_client.call(Shopping.tts_srv);         
                ROS_INFO("Go for cash");
                Shopping.move_base_client.sendGoal(goal_cash);
                Shopping.move_base_client.waitForResult();

                if(Shopping.move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    Shopping.tts_srv.request.cmd = "tts";
                    Shopping.tts_srv.request.txt = "reach the cash";
                    Shopping.tts_client.call(Shopping.tts_srv);
                    ROS_INFO("Succeeded go for cash !!!");
                }
                else
                {
                    Shopping.clear_costmaps_client.call(Shopping.ccm);
                    ROS_INFO("Once Again");
                    Shopping.move_base_client.sendGoal(goal_cash);
                    Shopping.move_base_client.waitForResult();
                    if(Shopping.move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        Shopping.tts_srv.request.cmd = "tts";
                        Shopping.tts_srv.request.txt = "reach the cash";
                        Shopping.tts_client.call(Shopping.tts_srv);
                    }
                }


            Shopping.navigation_to_goods = false;

        }
            ros::spinOnce();

    }//end while



return 0;
}
