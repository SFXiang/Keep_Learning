# AutoWare源码解析----lane_rule

**让我们一起来看lane_rule.cpp源码吧**

在代码中定义了topic

```
ros::Publisher traffic_pub;
ros::Publisher red_pub;
ros::Publisher green_pub;


traffic_pub = n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array",pub_waypoint_queue_size,pub_waypoint_latch);
red_pub = n.advertise<autoware_msgs::LaneArray>("/red_waypoints_array", pub_waypoint_queue_size,pub_waypoint_latch);
green_pub = n.advertise<autoware_msgs::LaneArray>("/green_waypoints_array", pub_waypoint_queue_size,pub_waypoint_latch);
```
按照我们目前实现的功能来说，只发布了/traffic_waypoints_array，原因如下：


在代码开头，定义了一个高精度地图all_vmap：
```
lane_planner::vmap::VectorMap all_vmap;
```
目前我们系统没有加载高精度地图，因此all_vmap是空的。（代码中绝大数函数都是围绕这高精度地图来修正我们的lane，但是...**目前阶段我们用不上**）。

在下面这段生成航点的代码中，有一个判断all_vmap是否为空，若空，则向/traffic_waypoints_array这个topic发布信息后，就直接return了。
```
//生成航点
void create_waypoint(const autoware_msgs::LaneArray& msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;
	cached_waypoint.lanes.clear();
	cached_waypoint.lanes.shrink_to_fit();
	for (const autoware_msgs::lane& l : msg.lanes)
		cached_waypoint.lanes.push_back(create_new_lane(l, header));
	if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty() ||
	    all_vmap.stoplines.empty() || all_vmap.dtlanes.empty()) {
		
		traffic_pub.publish(cached_waypoint);
		return; 
	}
...
...
}
```
#### 下面我们来看看源码上所订阅的topic
```
ros::Subscriber waypoint_sub = n.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);
ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
ros::Subscriber stopline_sub = n.subscribe("/vector_map_info/stop_line", sub_vmap_queue_size, cache_stopline);
ros::Subscriber dtlane_sub = n.subscribe("/vector_map_info/dtlane", sub_vmap_queue_size, cache_dtlane);
ros::Subscriber config_sub = n.subscribe("/config/lane_rule", sub_config_queue_size, config_parameter);
```

根据代码中所订阅的topic来看，除了**/lane_waypoints_array**和**/config/lane_rule**，其它都为vector_map_info的topic，该topic都由目前我们**没有用到的高精度地图**那块的node来发布的，所以目前都是没有任何消息的。不信我们可以查看其中的一个topic，比如说/vector_map_info/stop_line：

```
[~] rostopic info /vector_map_info/stop_line 17:38:06 
Type: vector_map_msgs/StopLineArray

Publishers: None

Subscribers: 
* /lane_rule (http://foredawn-TR:39064/)
```
可以发现Publishers为none，并没有发布者，因此我们获取不到topic的任何信息，其它几个topic也是如此，就不一一贴上来了。

**现在我们来看看目前订阅的topic即/lane_waypoints_array（/config/lane_rule也是从配置文件中读取，就不讨论了）**

我们先看/lane_waypoints_array是由谁发布的:
```
[~] rostopic info /lane_waypoints_array 17:38:25 
Type: autoware_msgs/LaneArray

Publishers: 
* /waypoint_loader (http://foredawn-TR:42982/)

Subscribers: 
* /lane_rule (http://foredawn-TR:39064/)
* /waypoint_marker_publisher (http://foredawn-TR:38376/)
```
可以看到发布的node为/waypoint_loader，我们去看看定义这个node的核心文件**waypoint_loader_core_cpp**，我们直接找到该代码中定义这个topic的那段代码
```
void WaypointLoaderNode::initPubSub()
{
  private_nh_.param<bool>("disable_decision_maker", disable_decision_maker_, true);
  // setup publisher
  if (disable_decision_maker_)
  {
    lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", 10, true);
  }
  else
  {
    lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_array", 10, true);
  }
  config_sub_ = nh_.subscribe("/config/waypoint_loader", 1, &WaypointLoaderNode::configCallback, this);
  output_cmd_sub_ =
      nh_.subscribe("/config/waypoint_loader_output", 1, &WaypointLoaderNode::outputCommandCallback, this);
}

```
我们向这个topic发布信息，但是这个信息我们是从我们订阅的一个topic中获取到的，即/config/waypoint_loader,我们来看看这个回调函数:

```
void WaypointLoaderNode::configCallback(const autoware_msgs::ConfigWaypointLoader::ConstPtr& conf)
{
  initParameter(conf);
  replanner_.initParameter(conf);

  multi_file_path_.clear();
  parseColumns(multi_lane_csv_, &multi_file_path_);
  autoware_msgs::LaneArray lane_array;
  createLaneArray(multi_file_path_, &lane_array);
  lane_pub_.publish(lane_array);
  output_lane_array_ = lane_array;
}

```
在这里我们可以看到形参为conf，所有的config都是由runtime manager发布的，在AutoWare图形化界面中，按照我们给定的路径读取到文件，再对数据通过一系列的处理，得到了laneArray信息。我们查看LaneArray.msg，可以看到数据类型为:
```
Header header
lane[] lanes
```
我们再查看lane.msg:
```
Header header
int32 increment
int32 lane_id
waypoint[] waypoints
```
其中的waypoints就是一系列的轨迹点，我们再接着查看waypoints.msg:
```
# global id
int32 gid 
# local id
int32 lid
geometry_msgs/PoseStamped pose   
geometry_msgs/TwistStamped twist  
dtlane dtlane
int32 change_flag
WaypointState wpstate
```
再接着看PoseStamped.msg:
```
Header header
Pose pose
```
关键的来了，即Pose出现了，我们查看Pose.msg，发现这是一个三维坐标系，表示车辆的位置信息：
```
float64 x
float64 y
float64 z
```
我们查看TwistStamped.msg，同理发现里面为:
```
Header header
Twist twist
```
查看Twist.msg，里面的linear和angular定义了车辆的线速度和角速度，与Pose结合详细定义了目前的车辆信息:
```
Vector3  linear
Vector3  angular
```
根据上面一系列msg类型，你可以了解到lane其实一个一系列带有坐标带有角速度和线速度的点。

**目前来说lane_rule这个node没有取到什么作用，它只是将/lane_waypoints_array这个topic发布的内容(laneArray)转发给了lane_select，当有了高精度地图的情况下，才有它真正有用的时候，我也仔细阅读了有高精度地图情况下的函数处理（根据高精度地图里的信息来修正目前的lane），等到时候有了高精度地图，再进行进一步研究。**