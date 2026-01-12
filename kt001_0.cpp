#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include "sophon_robot/pidConfig.h"

#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <vector>   // 确保引入 vector

#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_pid         0x12
#define sendType_params      0x13

using namespace std;
using namespace boost::asio;
	
io_service iosev;
serial_port sp(iosev);         

std::string port_name;
int baud_rate;
bool publish_odom_transform;
int kp;
int ki;
int kd;
double linear_correction;
double angular_correction;
      
ros::Time cmd_time;

double x = 0.0;
double y = 0.0;
double yaw = 0.0;

uint8_t checksum(uint8_t* buf, size_t len)
{
  uint8_t sum = 0x00;
  for(int i=0;i<len;i++)
  {
    sum += *(buf + i);
  }
  return sum;
}

void SetPID(int p,int i, int d)
{
	static uint8_t tmp[11];
	tmp[0] = head1; tmp[1] = head2; tmp[2] = 0x0b; tmp[3] = sendType_pid;
	tmp[4] = (p>>8)&0xff; tmp[5] = p&0xff;
	tmp[6] = (i>>8)&0xff; tmp[7] = i&0xff;
	tmp[8] = (d>>8)&0xff; tmp[9] = d&0xff;
	tmp[10] = checksum(tmp,10);
	write(sp,buffer(tmp,11));
}

void SetParams(double linear_correction,double angular_correction) {
	static uint8_t tmp[9];
	tmp[0] = head1; tmp[1] = head2; tmp[2] = 0x09; tmp[3] = sendType_params;
	tmp[4] = (int16_t)((int16_t)(linear_correction*1000)>>8)&0xff;
	tmp[5] = (int16_t)(linear_correction*1000)&0xff;
	tmp[6] = (int16_t)((int16_t)(angular_correction*1000)>>8)&0xff;
	tmp[7] = (int16_t)(angular_correction*1000)&0xff;
	tmp[8] = checksum(tmp,8);
	write(sp,buffer(tmp,9));
}

void SetVelocity(double x, double y, double yaw)
{
  static uint8_t tmp[11];
  tmp[0] = head1; tmp[1] = head2; tmp[2] = 0x0b; tmp[3] = sendType_velocity;
  tmp[4] = ((int16_t)(x*1000)>>8) & 0xff;
  tmp[5] = ((int16_t)(x*1000)) & 0xff;
  tmp[6] = ((int16_t)(y*1000)>>8) & 0xff;
  tmp[7] = ((int16_t)(y*1000)) & 0xff;
  tmp[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
  tmp[9] = ((int16_t)(yaw*1000)) & 0xff;
  tmp[10] = checksum(tmp,10);
  write(sp,buffer(tmp,11));
}

void cmd_callback(const geometry_msgs::Twist& msg)
{
  x = msg.linear.x;
  y = msg.linear.x; // 通常这里是linear.y，暂时保持和你源码一致
  yaw = msg.angular.z;
  cmd_time = ros::Time::now();
}

void pidConfig_callback(sophon_robot::pidConfig &config)
{
	kp = config.kp;
	ki = config.ki;
	kd = config.kd;
	SetPID(kp,ki,kd);
}

// 必须与 Arduino 的 parser.h 结构对齐
union FloatUnion {
    float fv;
    uint8_t cv[4];
};

void serial_task()
{
    // 定义状态机状态
    enum frameState {
        State_Head1, 
        State_Head2, 
        State_Data, 
        State_CheckSum
    };

    frameState state = State_Head1;
    
    // 协议定义：27字节 total
    // Head1(1) + Head2(1) + Data(24) + Checksum(1)
    const int PAYLOAD_LEN = 24; // 6个float * 4字节
    uint8_t data_buffer[PAYLOAD_LEN]; 
    uint8_t byte_in = 0;
    uint8_t check_sum_calc = 0;
    int data_idx = 0;

    // 数据变量
    float x_pos = 0.0;
    float y_pos = 0.0;
    float x_v = 0.0;
    float y_v = 0.0;
    float angular_v = 0.0;
    float pose_angular = 0.0;

    ros::NodeHandle n;
    
    // 移除 IMU 发布者，因为协议里没有 IMU 数据
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;

    // 预定义消息
    nav_msgs::Odometry odom_msgs;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;

    ROS_INFO("Start receiving custom protocol (Float32)...");

    while(ros::ok())
    {
        try {
            // 每次读取一个字节进行状态机流转
            boost::asio::read(sp, boost::asio::buffer(&byte_in, 1));

            switch (state)
            {
                case State_Head1:
                    if (byte_in == 0xAA) {
                        state = State_Head2;
                        check_sum_calc = 0; // 重置校验和
                        check_sum_calc += byte_in; // 累加 Head1
                    }
                    break;

                case State_Head2:
                    if (byte_in == 0x55) {
                        state = State_Data;
                        check_sum_calc += byte_in; // 累加 Head2
                        data_idx = 0; // 重置数据索引
                    } else {
                        state = State_Head1; // 匹配失败，回退
                    }
                    break;

                case State_Data:
                    data_buffer[data_idx] = byte_in;
                    check_sum_calc += byte_in; // 累加数据
                    data_idx++;
                    
                    if (data_idx >= PAYLOAD_LEN) {
                        state = State_CheckSum;
                    }
                    break;

                case State_CheckSum:
                    // 此时 byte_in 是收到的校验码
                    // 比较：收到的校验码 VS 我们计算的 (Head1+Head2+Data)
                    // 如果下位机的校验算法不同（例如只校验Data），请根据实际情况调整这里
                    if (byte_in == check_sum_calc) 
                    {
                        // === 校验通过，开始解析 ===
                        
                        // 利用 Union 进行字节到浮点数的转换 (Little Endian)
                        FloatUnion u_x_pos, u_y_pos, u_x_v, u_y_v, u_ang_v, u_pose_ang;

                        // 暴力填充：根据 send_data 结构体的顺序
                        // 0-3: x_pos
                        memcpy(u_x_pos.cv, &data_buffer[0], 4);
                        // 4-7: y_pos
                        memcpy(u_y_pos.cv, &data_buffer[4], 4);
                        // 8-11: x_v
                        memcpy(u_x_v.cv, &data_buffer[8], 4);
                        // 12-15: y_v
                        memcpy(u_y_v.cv, &data_buffer[12], 4);
                        // 16-19: angular_v
                        memcpy(u_ang_v.cv, &data_buffer[16], 4);
                        // 20-23: pose_angular
                        memcpy(u_pose_ang.cv, &data_buffer[20], 4);

                        // 赋值给变量
                        x_pos = u_x_pos.fv;
                        y_pos = u_y_pos.fv;
                        x_v   = u_x_v.fv;
                        y_v   = u_y_v.fv;
                        angular_v = u_ang_v.fv;
                        pose_angular = u_pose_ang.fv;

                        // === 发布 ROS 消息 ===
                        ros::Time now_time = ros::Time::now();

                        // 1. 发布 TF (odom -> base_link)
                        odom_trans.header.stamp = now_time;
                        odom_trans.header.frame_id = "odom";
                        odom_trans.child_frame_id = "base_link";
                        odom_trans.transform.translation.x = x_pos;
                        odom_trans.transform.translation.y = y_pos;
                        odom_trans.transform.translation.z = 0.0;
                        odom_quat = tf::createQuaternionMsgFromYaw(pose_angular);
                        odom_trans.transform.rotation = odom_quat;
                        odom_broadcaster.sendTransform(odom_trans);

                        // 2. 发布 Odom Topic
                        odom_msgs.header.stamp = now_time;
                        odom_msgs.header.frame_id = "odom";
                        odom_msgs.child_frame_id = "base_link";
                        
                        // 位置 (直接使用下位机发来的坐标)
                        odom_msgs.pose.pose.position.x = x_pos;
                        odom_msgs.pose.pose.position.y = y_pos;
                        odom_msgs.pose.pose.orientation = odom_quat;

                        // 速度 (直接使用下位机发来的 m/s, 不需要除以 dt)
                        odom_msgs.twist.twist.linear.x = x_v;
                        odom_msgs.twist.twist.linear.y = y_v;
                        odom_msgs.twist.twist.angular.z = angular_v;
                        
                        // 协方差矩阵 (根据需要调整)
                        odom_msgs.pose.covariance[0] = 0.01; // x
                        odom_msgs.pose.covariance[7] = 0.01; // y
                        odom_msgs.pose.covariance[35] = 0.01; // yaw
                        odom_msgs.twist.covariance[0] = 0.01; // vx
                        odom_msgs.twist.covariance[7] = 0.01; // vy
                        odom_msgs.twist.covariance[35] = 0.01; // vth

                        odom_pub.publish(odom_msgs);
                    } 
                    else 
                    {
                         // 校验失败，不打印过多日志以免刷屏
                         // ROS_WARN("Checksum failed: calc %x vs recv %x", check_sum_calc, byte_in);
                    }
                    
                    // 无论成功失败，重置状态
                    state = State_Head1;
                    break;
            }

        } catch (boost::system::system_error& e) {
            ROS_ERROR("Serial read error: %s", e.what());
            ros::Duration(1.0).sleep(); // 出错后暂停一下
        }
    }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "jetbot");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  pn.param<std::string>("port_name",port_name,std::string("/dev/ttyUSB0"));
  pn.param<int>("baud_rate",baud_rate,115200);
  pn.param<double>("linear_correction",linear_correction,1.0);
  pn.param<double>("angular_correction",angular_correction,1.0);
  pn.param<bool>("publish_odom_transform",publish_odom_transform,true);
  
  boost::system::error_code ec;
  sp.open(port_name,ec);
  if (ec) {
      ROS_ERROR("Cannot open port: %s", port_name.c_str());
      return -1;
  }
  sp.set_option(serial_port::baud_rate(baud_rate));   
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp.set_option(serial_port::parity(serial_port::parity::none));
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp.set_option(serial_port::character_size(8));
  
  ros::Subscriber cmd_sub = n.subscribe("/cmd_vel",10,cmd_callback);
  
  dynamic_reconfigure::Server<sophon_robot::pidConfig> server;
  dynamic_reconfigure::Server<sophon_robot::pidConfig>::CallbackType f;
  f = boost::bind(&pidConfig_callback, _1);
  server.setCallback(f);

  ros::Duration(0.02).sleep();
  SetParams(linear_correction,angular_correction);

  // 创建并分离线程
  thread serial_thread(boost::bind(serial_task));
  // 【关键修复】分离线程，防止主程序退出时 crash
  serial_thread.detach();
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  while(ros::ok()){
    current_time = ros::Time::now();
    if((current_time - last_time).toSec() > 0.02){           
      last_time = current_time;
      if((current_time - cmd_time).toSec() > 1) {
        x = 0.0; y = 0.0; yaw = 0.0;
      }
      SetVelocity(x,y,yaw);
    }
    ros::spinOnce();
    ros::Duration(0.001).sleep(); // 稍微释放CPU
  }
     
  return 0;
}
