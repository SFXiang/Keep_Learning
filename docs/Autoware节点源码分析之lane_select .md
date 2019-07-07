# Autoware节点源码分析之lane_select



节点lane_select主要对人车线路进行选择，该节点订阅话题如下 
```sh
  sub1_ = nh_.subscribe("traffic_waypoints_array", 1, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 1, &LaneSelectNode::callbackFromPoseStamped, this);
  sub3_ = nh_.subscribe("current_velocity", 1, &LaneSelectNode::callbackFromTwistStamped, this);
  sub4_ = nh_.subscribe("state", 1, &LaneSelectNode::callbackFromState, this);
  sub5_ = nh_.subscribe("/config/lane_select", 1, &LaneSelectNode::callbackFromConfig, this);
  sub6_ = nh_.subscribe("/decisionmaker/states", 1, &LaneSelectNode::callbackFromStates, this);
```

发步的话题如下//tuple_vec存储的是lane,closest_waypoint,change_flag 
```sh
 publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
 publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));
 publishChangeFlag(std::get<2>(tuple_vec_.at(current_lane_idx_)));
```

## 实例化对象 

 ```sh
lane_planner::LaneSelectNode lsn;
```

 进入构造函数，初始化对象的成员变量：
 ```sh
LaneSelectNode::LaneSelectNode()
  : private_nh_("~")
  , current_lane_idx_(-1)
  , right_lane_idx_(-1)
  , left_lane_idx_(-1)
  , is_lane_array_subscribed_(false)
  , is_current_pose_subscribed_(false)
  , is_current_velocity_subscribed_(false)
  , is_current_state_subscribed_(false)
  , is_config_subscribed_(false)
  , distance_threshold_(3.0)
  , lane_change_interval_(10.0)
  , lane_change_target_ratio_(2.0)
  , lane_change_target_minimum_(5.0)
  , vlength_hermite_curve_(10)
  , current_state_("UNKNOWN")
{
  initForROS();
}
```

initForROS()chushihua初始化 ROS中的一些参数，设置订阅和发布的函数

# 进入成员函数
```sh
void LaneSelectNode::run()
{
  ros::spin();
}
```
 ros::spin();ROS消息回调处理函数。ROS会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是立刻就被处理，而是必须要等到ros::spin()
 
 
 通过对源码的阅读，在订阅的回调函数中对传来的消息判断后主要逻辑功能集中在以下两个函数中
```sh
 initForLaneSelect();
 processing();
```
  -  initForLaneSelect();
```sh
void LaneSelectNode::initForLaneSelect()
{
  if (!isAllTopicsSubscribed())
    return;

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    publishClosestWaypoint(-1);
    resetLaneIdx();
    return;
  }

  findCurrentLane();//寻找距离当前p的最短距离的index
  findNeighborLanes();//发现领近线路   
  updateChangeFlag();//更新状态标识位
  createLaneForChange();//线路改变  

  publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
  publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));
  publishChangeFlag(std::get<2>(tuple_vec_.at(current_lane_idx_)));
  publishVisualizer();

  resetSubscriptionFlag();
  return;
}
```
  -  processing();
```sh
 void LaneSelectNode::processing()
{
  if (!isAllTopicsSubscribed())
    return;

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    publishClosestWaypoint(-1);
    resetLaneIdx();
    return;
  }

  // if closest waypoint on current lane is -1,
  if (std::get<1>(tuple_vec_.at(current_lane_idx_)) == -1)
  {
    publishClosestWaypoint(-1);
    resetLaneIdx();
    return;
  }

  findNeighborLanes();
  ROS_INFO("current_lane_idx: %d", current_lane_idx_);
  ROS_INFO("right_lane_idx: %d", right_lane_idx_);
  ROS_INFO("left_lane_idx: %d", left_lane_idx_);

  if (current_state_ == "LANE_CHANGE")
  {
    try
    {
      changeLane();
      std::get<1>(lane_for_change_) =
          getClosestWaypointNumber(std::get<0>(lane_for_change_), current_pose_.pose, current_velocity_.twist,
                                   std::get<1>(lane_for_change_), distance_threshold_);
      std::get<2>(lane_for_change_) = static_cast<ChangeFlag>(
          std::get<0>(lane_for_change_).waypoints.at(std::get<1>(lane_for_change_)).change_flag);
      ROS_INFO("closest: %d", std::get<1>(lane_for_change_));
      publishLane(std::get<0>(lane_for_change_));
      publishLaneID(std::get<0>(lane_for_change_));
      publishClosestWaypoint(std::get<1>(lane_for_change_));
      publishChangeFlag(std::get<2>(lane_for_change_));
    }
    catch (std::out_of_range)
    {
      ROS_WARN("Failed to get closest waypoint num\n");
    }
  }
  else
  {
    updateChangeFlag();
    createLaneForChange();

    publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
    publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));
    publishChangeFlag(std::get<2>(tuple_vec_.at(current_lane_idx_)));
  }
  publishVisualizer();
  resetSubscriptionFlag();
}
```

下面对两个函数中处理函数进行分析

```sh
void LaneSelectNode::findCurrentLane()//找到距离当前位置坐标点最短距离的index点（根据欧拉距离判断最小index）
{
  std::vector<uint32_t> idx_vec;
  idx_vec.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (std::get<1>(tuple_vec_.at(i)) == -1)
      continue;
    idx_vec.push_back(i);
  }
  current_lane_idx_ = findMostClosestLane(idx_vec, current_pose_.pose.position);//
}
```
发现当前lane的左右lane的index
```sh
void LaneSelectNode::findNeighborLanes()//寻找当前lane左右index并放入到对应左右index容器中
{
  int32_t current_closest_num = std::get<1>(tuple_vec_.at(current_lane_idx_));
  const geometry_msgs::Pose &current_closest_pose =
      std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.at(current_closest_num).pose.pose;//根据index在waypoints中找到最近的pose

  std::vector<uint32_t> left_lane_idx_vec;
  left_lane_idx_vec.reserve(tuple_vec_.size());
  std::vector<uint32_t> right_lane_idx_vec;
  right_lane_idx_vec.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (i == static_cast<uint32_t>(current_lane_idx_) || std::get<1>(tuple_vec_.at(i)) == -1)
      continue;

    int32_t target_num = std::get<1>(tuple_vec_.at(i));
    const geometry_msgs::Point &target_p = std::get<0>(tuple_vec_.at(i)).waypoints.at(target_num).pose.pose.position;

    geometry_msgs::Point converted_p = convertPointIntoRelativeCoordinate(target_p, current_closest_pose);//

    ROS_INFO("distance: %lf", converted_p.y);
    if (fabs(converted_p.y) > distance_threshold_)
    {
      ROS_INFO("%d lane is far from current lane...", i);
      continue;
    }

    if (converted_p.y > 0)
      left_lane_idx_vec.push_back(i);
    else
      right_lane_idx_vec.push_back(i);
  }

  if (!left_lane_idx_vec.empty())
    left_lane_idx_ = findMostClosestLane(left_lane_idx_vec, current_closest_pose.position);
  else
    left_lane_idx_ = -1;

  if (!right_lane_idx_vec.empty())
    right_lane_idx_ = findMostClosestLane(right_lane_idx_vec, current_closest_pose.position);
  else
    right_lane_idx_ = -1;
}
```

updateChangeFlag（）是对道路发生变化的一个状态位更新

### 重要！函数createLaneForChange()，看注释
```sh
void LaneSelectNode::createLaneForChange()//改变线路选择
{
  std::get<0>(lane_for_change_).waypoints.clear();
  std::get<0>(lane_for_change_).waypoints.shrink_to_fit();
  std::get<1>(lane_for_change_) = -1;

  const autoware_msgs::lane &cur_lane = std::get<0>(tuple_vec_.at(current_lane_idx_));//根据index找寻lane
  const int32_t &clst_wp = std::get<1>(tuple_vec_.at(current_lane_idx_));//根据index确定waypoint节点

  int32_t num_lane_change = getClosestLaneChangeWaypointNumber(cur_lane.waypoints, clst_wp);//得到最近距离的改变的waypoint
  ROS_INFO("num_lane_change: %d", num_lane_change);
  if (num_lane_change < 0 || num_lane_change >= static_cast<int32_t>(cur_lane.waypoints.size()))//判断当前lane是否拥有change状态标识
  {
    ROS_WARN("current lane doesn't have change flag");
    return;
  }

  if ((static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::right &&//判断当前是否有提供的路径可供人车改变lane选择
       right_lane_idx_ < 0) ||
      (static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::left &&
       left_lane_idx_ < 0))
  {
    ROS_WARN("current lane doesn't have the lane for lane change");
    return;
  }
/*以下注释代码主要工作 通过左右index 和 changeFlag 通过ros操作改变人车的lane选择*/
//   double dt = getTwoDimensionalDistance(cur_lane.waypoints.at(num_lane_change).pose.pose.position,
//                                         cur_lane.waypoints.at(clst_wp).pose.pose.position);
//   double dt_by_vel = current_velocity_.twist.linear.x * lane_change_target_ratio_ > lane_change_target_minimum_ ?
//                          current_velocity_.twist.linear.x * lane_change_target_ratio_ :
//                          lane_change_target_minimum_;
//   ROS_INFO("dt : %lf, dt_by_vel : %lf", dt, dt_by_vel);
//   autoware_msgs::lane &nghbr_lane =
//       static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::right ?
//           std::get<0>(tuple_vec_.at(right_lane_idx_)) :
//           std::get<0>(tuple_vec_.at(left_lane_idx_));
//   const int32_t &nghbr_clst_wp =
//       static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::right ?
//           std::get<1>(tuple_vec_.at(right_lane_idx_)) :
//           std::get<1>(tuple_vec_.at(left_lane_idx_));

//   int32_t target_num = -1;
//   for (uint32_t i = nghbr_clst_wp; i < nghbr_lane.waypoints.size(); i++)
//   {
//     if (i == nghbr_lane.waypoints.size() - 1 ||
//         dt + dt_by_vel < getTwoDimensionalDistance(nghbr_lane.waypoints.at(nghbr_clst_wp).pose.pose.position,
//                                                    nghbr_lane.waypoints.at(i).pose.pose.position))
//     {
//       target_num = i;
//       break;
//     }
//   }

//   ROS_INFO("target_num : %d", target_num);
//   if (target_num < 0)
//     return;

//   std::get<0>(lane_for_change_).header.stamp = nghbr_lane.header.stamp;
//   std::vector<autoware_msgs::waypoint> hermite_wps = generateHermiteCurveForROS(
//       cur_lane.waypoints.at(num_lane_change).pose.pose, nghbr_lane.waypoints.at(target_num).pose.pose,
//       cur_lane.waypoints.at(num_lane_change).twist.twist.linear.x, vlength_hermite_curve_);

//   for (auto &&el : hermite_wps)
//     el.change_flag = cur_lane.waypoints.at(num_lane_change).change_flag;

//   std::get<0>(lane_for_change_).waypoints.reserve(nghbr_lane.waypoints.size() + hermite_wps.size());
//   std::move(hermite_wps.begin(), hermite_wps.end(), std::back_inserter(std::get<0>(lane_for_change_).waypoints));
//   auto itr = nghbr_lane.waypoints.begin();
//   std::advance(itr, target_num);
//   for (auto i = itr; i != nghbr_lane.waypoints.end(); i++)
//   {
//     if (getTwoDimensionalDistance(itr->pose.pose.position, i->pose.pose.position) < lane_change_interval_)
//       i->change_flag = enumToInteger(ChangeFlag::straight);
//     else
//       break;
//   }
//   std::copy(itr, nghbr_lane.waypoints.end(), std::back_inserter(std::get<0>(lane_for_change_).waypoints));
// }
```

### 总结
lane_select 节点的设计主要是为人车在行驶的过程中当前路径的左右还有其他路径时，人车根据index，changeFlag状态标识，判断车辆改变的方向是选择左还是右，之后车辆作出相应的动作变化。（个人理解欢迎指正）
