
#IMU 测量单元用于ndt_matching中


IMU的全称是inertial measurement unit，即惯性测量单元，通常由陀螺仪、加速剂和算法处理单元组成，通过对加速度和旋转角度的测量得出自体的运动轨迹。IMU只提供相对定位信息，即提供自体从某时刻开始相对于某个起始位置的运动轨迹和姿态。然而，将IMU的相对定位与RTK GPS的绝对定位进行融合后，就产生了两个无可替代的优点：

* IMU可以验证RTK GPS结果的自洽性，并对无法自洽的绝对定位数据进行滤波和修正。
* IMU可以在RTK GPS信号消失之后，仍然提供持续若干秒的亚米级定位精度，为自动驾驶汽车争取宝贵的异常处理的时间。

同样的道理，IMU也可以在相对定位失效时，对相对定位的结果进行航迹推演，在一段时间内保持相对定位的精度；例如，在车道线识别模块失效时，利用失效之前感知到的道路信息和IMU对汽车航迹的推演，仍然能够让汽车继续在车道内行驶。
GPS+IMU作为主流的高精定位方法，IMU不仅可以在GPS信号消失的时候，接管绝对定位，并且提供相对位置和相对姿态，还可以在GPS信号发生漂移的时候对GPS信号进行纠偏，众所周知，在某些地区，特别是地震带上的GPS漂移已经成为自动驾驶高精地图和高精定位领域的一个难题。

下面我们可以看看在ndt_matching节点中，关于use_imu的使用：

首先是imu_odom_calc(ros::Time current_time)函数,该段函数主要通过的到当前时间计算出在odom（里程计坐标系）和IMU工作域下的位置信息：
```
static void imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  predict_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  predict_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  predict_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  predict_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  predict_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  predict_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}
```
函数imu_calc(ros::Time current_time)：计算在IMU工作区域的坐标位置信息
```
static void imu_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  predict_pose_imu.x = previous_pose.x + offset_imu_x;
  predict_pose_imu.y = previous_pose.y + offset_imu_y;
  predict_pose_imu.z = previous_pose.z + offset_imu_z;
  predict_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  predict_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  predict_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
```
函数odom_calc(ros::Time current_time)：计算在odom下的坐标位置信息
```
static void odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  predict_pose_odom.x = previous_pose.x + offset_odom_x;
  predict_pose_odom.y = previous_pose.y + offset_odom_y;
  predict_pose_odom.z = previous_pose.z + offset_odom_z;
  predict_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  predict_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  predict_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}
```

根据ndt_matching 勾选的参数 将哪一类型的参数返回给ndt的预测位置代码如下：
```
pose predict_pose_for_ndt;
    if (_use_imu == true && _use_odom == true)
      predict_pose_for_ndt = predict_pose_imu_odom;
    else if (_use_imu == true && _use_odom == false)
      predict_pose_for_ndt = predict_pose_imu;
    else if (_use_imu == false && _use_odom == true)
      predict_pose_for_ndt = predict_pose_odom;
    else
      predict_pose_for_ndt = predict_pose;
```
