#Autoware软件架构
* 从话题  localizer_pose                                  订阅     定位器的姿态信息        ：geometry_msgs/PoseStamped.msg
* 从话题  current_pose                                     订阅     控制器的姿态信息        ：geometry_msgs/PoseStamped.msg
* 从话题  vscan_points                                    订阅      vscan反馈的点云信息：sensor_msgs/PointCloud2.msg，并用 pcl 将点云转换为 XYZ 空间坐标系的数据结构，然后过滤掉“指定探测范围(g_detection_height_top，g_detection_height_bottom)之外”的点和“坐标系中心点”，这些是无用数据
* 从话题  base_waypoints                              订阅     道路航图基准点 信息    ：autoware_msgs/lane.msg(current_waypoints_)
* 从话题  current_velocity                              订阅     当前速度信息                  ：geometry_msgs/TwistStamped.msg(g_current_vel：只获取的速度信息中的 x 轴的线速度)
* 从话题  configCallback                                 订阅     本节点初始参数信息     ：autoware_msgs/ConfigLatticeVelocitySet.msg(g_others_distance：限定停在障碍物前的距离，g_decel：限定减速度，g_detection_range：限定障碍物探测范围.....，除此之外在节点主程序中这些参数已被赋予缺省值)
* 从话题  vector_map_info/cross_walk     订阅     人行道数据                      ：vector_map/CrossWalkArray.msg(crosswalk_，包括一组CrossWalk{id,aid,type,bdid,linkid}和一个header)
* 从话题  vector_map_info/area                   订阅     道路区域信息                 ：vector_map/AreaArray.msg(area_，包括一组Area{aid,slid,elid}和一个header)
* 从话题  vector_map_info/line                     订阅    道路基准点信息             ：vector_map/LineArray.msg(line_，包括一组Line{lid,bpid,fpid,blid,flid}和一个header)
* 从话题  vector_map_info/point                  订阅     道路点信息                 ：vector_map/PointArray.msg(point_，包括一组Point{pid,b,l,h,bx,ly,ref,mcode1,mcode2,mcode3}和一个header)

* 向话题  detection                                              发布     使用矢量数据标示的探测范围信息            ：visualization_msgs/MarkerArray.msg
* 向话题  sound_player                                     发布      设置声音播放设备                                          ：std_msgs/String.msg
* 向话题  temporal_waypoints                      发布      道路瞬时基准点                                               ：autoware_msgs/lane.msg
* 向话题 closet_waypoint                               发布      最近的道路瞬时基准点                                   ：std_msgs/Int32.msg
* 向话题  obstacle                                               发布      障碍物标识信息                                                ：visualization_msgs/Marker.msg

* 设定回调信息频率为 LOOP_RATE(10Hz)

##人行横道
* 1.确保，矢量地图的数据订阅到(load_all=true)
* 2.确保，从订阅的人行道数据CrossWalkArray.msg(其中包含斑马线、非机动车道、非机动停车位)中提取出属于斑马线的数据，getAID

         map < bdid , vector < aid > > ，vector < bdid >
* 3.图中找面，用2的bdid和aid，使用aid在订阅的道路区域信息AreaArray.msg找到对应Area的slid ( =lid )，作为下一步的起点

         for ( bdid ) {
                  for ( aid ) {
                          lid=Area { aid } .slid 
                  }
          }
* 4.面中找线，用3的lid作为开始在订阅的道路线数据LineArray.msg中按接力式的方式搜寻Line，使用bpid作为下一步的条件
* 5.线中找点，用4的bpid作为开始在订阅的道路点数据PointArray.msg，搜索对应的点值 Point ( ly , bx ,h )，并存储为点集vector&#60;Point&#62;
* 6.第一步，用5的到的的点集筛选两点间距最大的两个点对&#60;Point,Point&#62;，求该aid区域的重心 Point++/4，计算该bdid下的所有aid区域的重心并存储在     detection_points_[bdid].( vector&#60;Point&#62; )；第二步，使用5的点集求距离最大的两个点之间的距离，为该区域斑马线的宽度，将该bdid下的所有区域马线宽度求平均，并存储到detection_points_[bdid].( width )；第三步，将第一步得到的所有重心取平均计算中心点，并存储在detection_points[bdid].( center )。
 
         vector<Point>
         while(lid){
                  for( Line ){
                           push.( Point { Line{ lid }.bpid } )
                           lid=Line{ lid }.flid
                  }
         }
        centerofGravity  ( Point1 + Point2 + Point3 + Point4 ) / 4
         crosswalkWidth=max_length(Point1,Point2)
* 7.至此人行道的检测计算完成，所有数据存储到了数据结构 unordered_map&#60;int ,CrossWalkPoints&#62;detection_points_ 中。

##道路检测
* 1.获取订阅到的道路基准点信息LineArray.msg，以及控制器姿态信息
* 2.判断所有航道基准点和车辆当前的位置相距是否在搜寻范围之内
* 3.判断所有航道基准点是否在车辆的前面
* 4.判断所有航道基准点和车辆的夹角是否在规定的角度阀值
* 5.如果以上条件都成立，则将符合条件的巷道基准点存入航道点候选数组中 vector&#60; int &#62;waypoint_candidate
* 6.在5中得到的候选航道基准点筛选离车辆当前位置最近的点，如果没有候选点，则在回到2，在搜寻范围之外找最近的航道基准点
* 7.向话题 closet_waypoint 发布最近的道路瞬时基准点

##人行道检测和道路检测都完成后开始斑马线检测
* 1.遍历检测最近航道点到限制距离之内的航道点

          for (int num = g_closest_waypoint; num < g_closest_waypoint + g_search_distance; num++)
* 2.遍历人行道检测过程中(2)产生的 vector&#60;bdid&#62; ，判断每个bdid对应的 detection_points_ 内所有 center 到(1)中的航道点之间的距离，过滤掉大于限定距离 ignore_distance 的bdid
* 3.检测在限定距离 find_distance 内是否发现斑马线，并保存bdid，否则返回 -1

##障碍物检测
1. 通过障碍物检测判断下一个动作是保持，加速，停止，或是其他动作
 
          EControl detection_result = obstacleDetection()
          EControl obstacleDetection()
         {
                static int false_count = 0;
                static EControl prev_detection = KEEP;
                //获取vscan检测的结果
               EControl vscan_result = vscanDetection();
              //显示检测范围
               displayDetectionRange(vmap.getDetectionCrossWalkID(), g_closest_waypoint, vscan_result);

               if (prev_detection == KEEP){
                             if (vscan_result != KEEP){ 
                                         // found obstacle
                                         // 显示障碍物
                                         displayObstacle(vscan_result);
                                         prev_detection = vscan_result;
                                         // SoundPlay();
                                         false_count = 0;
                                        return vscan_result;
                                       }else{ 
                                              // no obstacle
                                             prev_detection = KEEP;
                                              return vscan_result;
                                       }
              }else{  
                           // prev_detection = STOP or DECELERATE
                          if (vscan_result != KEEP){  
                                            // found obstacle
                                           displayObstacle(vscan_result);
                                           prev_detection = vscan_result;
                                           false_count = 0;
                                           return vscan_result;
                             }else{  
                                           // no obstacle
                                            false_count++;

                                            // fail-safe
                                           if (false_count >= LOOP_RATE / 2){
                                                     g_obstacle_waypoint = -1;
                                                     false_count = 0;
                                                    prev_detection = KEEP;
                                                     return vscan_result;
                                            }else{
                                                     displayObstacle(OTHERS);
                                                     return prev_detection;
                                           }
                            }
                  }
        }
2. 对于不同结果设置不同的处理措施
                      void changeWaypoint(EControl detection_result)
                      {
                                int obs = g_obstacle_waypoint;

                                 if (detection_result == STOP)
                                 {  // STOP for obstacle，计算计划停止的位置航道点，改变行驶计划，避免突然减速，设置下一个动作为止航道点，发布规划的航道点
                                    // stop_waypoint is about g_others_distance meter away from obstacles
                                    int stop_waypoint = obs - ((int)(g_others_distance / g_path_change.getInterval()));
                                   // change waypoints to stop by the stop_waypoint
                                      g_path_change.changeWaypoints(stop_waypoint);
                                      g_path_change.avoidSuddenBraking();
                                      g_path_change.setTemporalWaypoints();
                                      g_temporal_waypoints_pub.publish(g_path_change.getTemporalWaypoints());
                                    }
                                      else if (detection_result == DECELERATE)
                                        {  // DECELERATE for obstacles，设定航道点，设定减速机制，是指下一个动作位置航道点，发布规划的航道点
                                          g_path_change.setPath(g_path_dk.getCurrentWaypoints());
                                            g_path_change.setDeceleration();
                                           g_path_change.setTemporalWaypoints();
                                          g_temporal_waypoints_pub.publish(g_path_change.getTemporalWaypoints());
                                        }
                                       else
                                          {  // ACELERATE or KEEP，设定航道点，避免突然加速，避免突然刹车，设置下一个动作位置的航道点，发布
                                             //  规划的行驶航道点
                                             g_path_change.setPath(g_path_dk.getCurrentWaypoints());
                                              g_path_change.avoidSuddenAceleration();
                                             g_path_change.avoidSuddenBraking();
                                               g_path_change.setTemporalWaypoints();
                                             g_temporal_waypoints_pub.publish(g_path_change.getTemporalWaypoints());
                                          }