#AutoWare源码解析——twist_filter节点
使用到的消息格式：
geometry_msgs::TwistStamped 消息格式 pure_pursuit节点发布的车辆运动信息
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
```
autoware_msgs::ConfigTwistFilter 配置文件消息格式，runtime manager发送的消息，对节点中的参数进行设置

```
Header header
float32 lateral_accel_limit
float32 lowpass_gain_linear_x
float32 lowpass_gain_angular_z
```
该节点的主要功能就是对pure_suit节点输出的汽车运动速度进行低通滤波，消除杂波使速度更加平滑。低通滤波算法如下： 

                               Yn=a* Xn+(1-a) *Yn-1

式中 Xn——本次采样值；Yn-1——上次的滤波输出值;a——滤波系数，其值通常远小于1;Yn——本次滤波的输出值。 
由上式可以看出，本次滤波的输出值主要取决于上次滤波的输出值(注意不是上次的采样值，这和加权平均滤波是有本质区别的)，本次采样值对滤波输出的贡献是比较小的，但多少有些修正作用，这种算法便模拟了具体有教大惯性的低通滤波器功能。

节点代码
```
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <iostream>

#include "autoware_msgs/ConfigTwistFilter.h"

namespace {

//Publisher
ros::Publisher g_twist_pub;
double g_lateral_accel_limit = 5.0;//设置无人车的最大侧向加速度
double g_lowpass_gain_linear_x = 0.0;//设置x方向线速度低通滤波器的增益
double g_lowpass_gain_angular_z = 0.0;//设置角速度方向低通滤波器的增益
constexpr double RADIUS_MAX = 9e10;//最大转弯半径
constexpr double ERROR = 1e-8;
//通过回调函数设置滤波器的参数
void configCallback(const autoware_msgs::ConfigTwistFilterConstPtr &config)
{
  g_lateral_accel_limit = config->lateral_accel_limit;
  ROS_INFO("g_lateral_accel_limit = %lf",g_lateral_accel_limit);
  g_lowpass_gain_linear_x = config->lowpass_gain_linear_x;
  ROS_INFO("lowpass_gain_linear_x = %lf",g_lowpass_gain_linear_x);
  g_lowpass_gain_angular_z = config->lowpass_gain_angular_z;
  ROS_INFO("lowpass_gain_angular_z = %lf",g_lowpass_gain_angular_z);
}

void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{

  double v = msg->twist.linear.x;
  double omega = msg->twist.angular.z;
  //若角速度接近于0则直接放送消息
  if(fabs(omega) < ERROR){
    g_twist_pub.publish(*msg);
    return;
  }
  //计算当前角速度下的方向最大线速度
  double max_v = g_lateral_accel_limit / omega;

  geometry_msgs::TwistStamped tp;
  tp.header = msg->header;
  //当前测量的侧向加速度
  double a = v * omega;
  ROS_INFO("lateral accel = %lf", a);

  //如果当前侧向加速度大于最大侧向加速度，就将x方向线速度设置为max_v
  tp.twist.linear.x = fabs(a) > g_lateral_accel_limit ? max_v
                    : v;
  tp.twist.angular.z = omega;

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  //对线速度和角速度进行低通滤波
  lowpass_linear_x = g_lowpass_gain_linear_x * lowpass_linear_x + (1 - g_lowpass_gain_linear_x) * tp.twist.linear.x;
  lowpass_angular_z = g_lowpass_gain_angular_z * lowpass_angular_z + (1 - g_lowpass_gain_angular_z) * tp.twist.angular.z;

  tp.twist.linear.x = lowpass_linear_x;
  tp.twist.angular.z = lowpass_angular_z;

  ROS_INFO("v: %f -> %f",v,tp.twist.linear.x);
  g_twist_pub.publish(tp);

}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_filter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    //订阅pure_pursuit节点的消息
    ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, TwistCmdCallback);
    //订阅runtime_manager节点的消息，进行参数设置
    ros::Subscriber config_sub = nh.subscribe("config/twist_filter", 10, configCallback);
    g_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);

    ros::spin();
    return 0;
}


```