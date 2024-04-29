#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/Imu.h>


// Useful customized headers
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint *_trajGene = new TrajectoryGeneratorWaypoint();
AstarPathFinder *_astar_path_finder = new AstarPathFinder();

// Set the obstacle map
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub, _imu_sub;
ros::Publisher _traj_vis_pub, _traj_pub, _path_vis_pub;

// for planning
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel, start_acc, imu_acc;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;
bool has_imu = false;

// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ
} exec_state = STATE::INIT;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
void execCallback(const ros::TimerEvent &e);

// declare
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void trajOptimization(Eigen::MatrixXd path);
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);
void rcvImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
void trajPublish(MatrixXd polyCoeff, VectorXd time);
bool trajGeneration();
VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);
Vector3d getAcc(double t_cur);

// change the state to the new state
// 用于改变当前执行状态（例如从等待目标转到生成新轨迹），并输出状态改变的日志信息
void changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

// 打印当前的执行状态，帮助监控节点的运行状态
void printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

// 接收来自里程计的数据，更新无人机的当前位置和速度信息
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  odom_pt(0) = odom->pose.pose.position.x;
  odom_pt(1) = odom->pose.pose.position.y;
  odom_pt(2) = odom->pose.pose.position.z;

  odom_vel(0) = odom->twist.twist.linear.x;
  odom_vel(1) = odom->twist.twist.linear.y;
  odom_vel(2) = odom->twist.twist.linear.z;

  has_odom = true;
}

void rcvImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  imu_acc(0) = imu->linear_acceleration.x;
  imu_acc(1) = imu->linear_acceleration.y;
  imu_acc(2) = imu->linear_acceleration.z;

  has_imu = true;

}

// Control the State changes
//定时器回调函数，定期检查和更新无人机的状态，决定是否需要生成新的轨迹或重新规划现有轨迹。
//根据当前的状态（如是否有目标位置，是否有里程计数据）和时间条件（如轨迹执行的持续时间），控制状态机的转换。
void execCallback(const ros::TimerEvent &e) {
  static int num = 0;
  num++;
  if (num == 100) {
    printState();
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  switch (exec_state) {
  case INIT: {
    if (!has_odom)
      return;
    if (!has_target)
      return;
    changeState(WAIT_TARGET, "STATE");
    break;
  }

  case WAIT_TARGET: {
    if (!has_target)
      return;
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case GEN_NEW_TRAJ: {
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case EXEC_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_replan = ros::Duration(1, 0).toSec();
    t_cur = min(time_duration, t_cur);

    // 如果当前时间接近或超过了轨迹的总持续时间，这表示轨迹执行即将完成或已完成。
    // 设置has_target为false，表明当前没有新的目标，然后将状态改变为WAIT_TARGET，等待新的目标设置
    if (t_cur > time_duration - 1e-2) {
      has_target = false;
      changeState(WAIT_TARGET, "STATE");
      return;
    } 
    // 当前位置接近目标位置，不进行任何操作，继续执行当前轨迹
    else if ((target_pt - odom_pt).norm() < no_replan_thresh) {
      return;
    } 
    // 当前位置仍然非常接近起点，不进行任何操作
    else if ((start_pt - odom_pt).norm() < replan_thresh) {
      return;
    } 
    // 当前时间小于设定的重规划时间间隔，则不进行重规划
    else if (t_cur < t_replan) {
      return;
    } 
    // 默认的重规划触发
    else {
      changeState(REPLAN_TRAJ, "STATE");
    }
    break;
  }
  case REPLAN_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_delta = ros::Duration(0, 50).toSec();
    t_cur = t_delta + t_cur;
    start_pt = getPos(t_cur);
    start_vel = getVel(t_cur);
    start_acc = getAcc(t_cur);
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }
  }
}

// 接收目标航点，更新目标位置，并根据当前状态触发轨迹生成或重规划
void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
  if (wp.poses[0].pose.position.z < 0.0)
    return;
  target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y,
      wp.poses[0].pose.position.z;
  ROS_INFO("[node] receive the planning target");
  start_pt = odom_pt;
  start_vel = odom_vel;
  start_acc = imu_acc;
  has_target = true;

  if (exec_state == WAIT_TARGET)
    changeState(GEN_NEW_TRAJ, "STATE");
  else if (exec_state == EXEC_TRAJ)
    changeState(REPLAN_TRAJ, "STATE");
}

// 接收来自传感器（如激光雷达）的点云数据，用于更新障碍物信息
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;

  pcl::fromROSMsg(pointcloud_map, cloud);

  if ((int)cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;
  for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
    pt = cloud.points[idx];
    // set obstalces into grid map for path planning
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);
  }
}

// trajectory generation: front-end + back-end
// front-end : A* search method
// back-end  : Minimum snap trajectory generation
// 负责整个轨迹生成过程，包括调用A*算法进行路径搜索，使用RDP算法简化路径，
// 以及调用轨迹优化函数生成最终轨迹。最后将生成的轨迹发布出去，并返回轨迹生成是否成功
bool trajGeneration() {
  /**
   *
   * STEP 1:  search the path and get the path
   *
   * **/
  _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
  auto grid_path = _astar_path_finder->getPath();

  // Reset map for next call

  /**
   *
   * STEP 2:  Simplify the path: use the RDP algorithm
   *
   * **/
  grid_path = _astar_path_finder->pathSimplify(grid_path, _path_resolution);
  MatrixXd path(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k];
  }

  /**
   *
   * STEP 3:  Trajectory optimization
   *
   * **/
  trajOptimization(path);
  time_duration = _polyTime.sum();

  // Publish the trajectory
  trajPublish(_polyCoeff, _polyTime);
  // record the trajectory start time
  time_traj_start = ros::Time::now();
  // return if the trajectory generation successes
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

// 对简化后的路径进行轨迹优化，生成一个满足动力学和安全性要求的平滑轨迹。包括时间分配和多项式轨迹生成
void trajOptimization(Eigen::MatrixXd path) {
  // if( !has_odom ) return;
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);

  vel.row(0) = start_vel;
  acc.row(0) = start_acc;

  /**
   *
   * STEP 3.1:  finish the timeAllocation() using resonable allocation
   *
   * **/
  _polyTime = timeAllocation(path);

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  // ROS_INFO("test PolyQPGeneration");
  _polyCoeff =
      _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

  // check if the trajectory is safe, if not, do reoptimize
  int unsafe_segment;

  /**
   *
   * STEP 3.3:  finish the safeCheck()
   *
   * **/
  unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);

  MatrixXd repath = path;
  int count = 0;
  while (unsafe_segment != -1) {
    /**
     *
     * STEP 3.4:  reoptimize
     * the method comes from: "Polynomial Trajectory
     * Planning for Aggressive Quadrotor Flight in Dense Indoor Environment"
     * part 3.5
     *
     * **/
    //这里要在有碰撞的区间内插点
    //Vector<Vector3d> grid_path里包含了所有waypoint
    Vector3d start_point = repath.row(unsafe_segment);
    Vector3d end_point = repath.row(unsafe_segment + 1);

    // 执行A*搜索
    _astar_path_finder->AstarGraphSearch(start_point, end_point);
    auto new_grid_path = _astar_path_finder->getPath();
    ROS_INFO("\033[35m============re-astar %d==============\033[0m",count);
    
    // 使用RDP算法简化路径
    new_grid_path = _astar_path_finder->pathSimplify(new_grid_path, _path_resolution/1.5);
    MatrixXd new_path(new_grid_path.size(), 3);
    for (int k = 0; k < int(new_grid_path.size()); k++) {
      new_path.row(k) = new_grid_path[k];
    }
    ROS_INFO("\033[35m============re-simplify %d==============\033[0m",count);

    // 替换原路径中不安全的段
    // 注意：这里可能需要更复杂的逻辑来处理路径数组的拼接
    if (new_path.rows() > 2) {
      // 重新构建完整路径，需要删除不安全段之间的旧路径，插入新路径
      MatrixXd temp_path(repath.rows() - 2 + new_path.rows(), 3);
      temp_path << repath.block(0, 0, unsafe_segment + 1, 3),  // 保留原路径的起始段
                  new_path.block(1, 0, new_path.rows() - 2, 3),  // 插入新路径（去除首尾重复点）
                  repath.block(unsafe_segment + 1, 0, repath.rows() - unsafe_segment - 1, 3);  // 保留原路径的结束段
      repath = temp_path;
    }
    ROS_INFO("\033[35m============re-path %d==============\033[0m",count);
    _polyTime = timeAllocation(repath);
    _polyCoeff =
      _trajGene->PolyQPGeneration(_dev_order, repath, vel, acc, _polyTime);
    ROS_INFO("\033[35m============re-caculate C and T %d==============\033[0m",count);

    // 重新检查路径安全性
    unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);  // 假设safeCheck现在可以处理整个路径并返回不安全的段索引
    ROS_INFO("\033[35m============re-safecheck %d==============\033[0m",count);

    count++;
    if (count > 10) {  // 防止无限循环
      ROS_WARN("Reached maximum number of re-planning attempts");
      break;
    }

  }
  // visulize path and trajectory
  visPath(repath);
  visTrajectory(_polyCoeff, _polyTime);
}

// 将优化后的轨迹发布到ROS主题
void trajPublish(MatrixXd polyCoeff, VectorXd time) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
             "publish.");
    return;
  }

  unsigned int poly_number;

  static int count =
      1; // The first trajectory_id must be greater than 0. zxzxzxzx

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  // traj_msg.header.frame_id = std::string("/world");
  traj_msg.header.frame_id = std::string("world");
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 2 * _dev_order - 1; // the order of polynomial
  traj_msg.num_segment = time.size();

  Vector3d initialVel, finalVel;
  initialVel = _trajGene->getVelPoly(_polyCoeff, 0, 0);
  finalVel = _trajGene->getVelPoly(_polyCoeff, traj_msg.num_segment - 1,
                                   _polyTime(traj_msg.num_segment - 1));
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));

  poly_number = traj_msg.num_order + 1;
  // cout << "p_order:" << poly_number << endl;
  // cout << "traj_msg.num_order:" << traj_msg.num_order << endl;
  // cout << "traj_msg.num_segment:" << traj_msg.num_segment << endl;
  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) *
                                pow(time(i), j));
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) *
                                pow(time(i), j));
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
  }
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}

// 计算在给定距离、速度和加速度下，使用梯形速度剖面所需的时间
double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}
// 对于给定的路径，分配每段路径的飞行时间
VectorXd timeAllocation(MatrixXd Path) {
  // VectorXd time(Path.rows() - 1);
  int n_segments = Path.rows() - 1;
  VectorXd time(n_segments);

  double total_distance = 0.0;
  VectorXd segment_lengths(n_segments);
  for(int i=0; i<n_segments; i++){
    segment_lengths(i) = (Path.row(i+1) - Path.row(i)).norm();
    total_distance += segment_lengths(i);
    time(i) = timeTrapzVel(segment_lengths(i), _Vel, _Acc);
    ROS_INFO("\033[35msegment(%d) = %f ,time(%d) = %f\033[0m", i, segment_lengths(i), i, time(i));
  }
  double t_acc = _Vel / _Acc;
  double s_acc = 0.5 * _Acc * std::pow(t_acc, 2);
  std::cout << "s_acc = " << s_acc << std::endl;
  std::cout << "total_distance = " << total_distance << std::endl;

  // if(total_distance <= 2 * s_acc){
  //   double t_all_half = std::sqrt(total_distance / _Acc);
  //   double t_now = 0.0;
  //   double v_now = 0.0;
  //   bool acc_flag = false;
  //   for(int i=0; i<n_segments; i++){
  //     if(t_now < t_all_half){
  //       double t_assumue = (-v_now + std::sqrt(std::pow(v_now,2) + 2*_Acc*segment_lengths(i)))/(_Acc);
  //       if(t_now + t_assumue <= t_all_half){
  //         time(i) = t_assumue;
  //         t_now += t_assumue;
  //         v_now += _Acc * t_assumue;
  //       }
  //       else if(t_now + t_assumue > t_all_half){
  //         double t_up = t_all_half - t_now;
  //         double rest_dis = segment_lengths(i) - (v_now * t_up + 0.5 *_Acc * std::pow(t_up,2));
  //         ROS_INFO("rest_dis = %d",rest_dis);
  //         double v_max = v_now + _Acc * t_up;
  //         double t_down = (v_max - std::sqrt(std::pow(v_max,2) - 2*_Acc*rest_dis))/(_Acc);
  //         t_now += (t_up + t_down);
  //         v_now += v_max -_Acc * t_down;
  //         time(i) = t_up + t_down;
  //       }
  //     }
  //     else if(t_now >= t_all_half){
  //       double t_assume = (v_now - std::sqrt(std::pow(v_now,2) - 2*_Acc*segment_lengths(i)))/(_Acc);
  //       if(t_now + t_assume <= 2*t_all_half){
  //         time(i) = t_assume;
  //         t_now += t_assume;
  //         v_now -= _Acc * t_assume;
  //       }
  //       else{
  //         double t_end = v_now/_Acc;
  //         time(i) = t_end;
  //         t_now += t_end;
  //         v_now -= _Acc * t_end;
  //       }
  //     }
      
  //   }
  // }
  // else if(total_distance > 2 * s_acc){

  // }

  // for(int i=0; i<n_segments; i++){

  //   double t_sum = total_distance / (1.0 * _Vel);
  //   time(i) = (t_sum * segment_lengths(i)) / (total_distance);
  //   std::cout << "time" << i << " = " << time(i) <<std::endl;
  // }
  return time;
}

// 可视化生成的轨迹，将轨迹以3D点的形式发布，用于在RViz等工具中显示
void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  // _traj_vis.header.frame_id = "/world";
  _traj_vis.header.frame_id = "world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 0.0;
  _traj_vis.color.g = 0.5;
  _traj_vis.color.b = 1.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub.publish(_traj_vis);
}

// 可视化规划的路径点，同样是发布为3D点云，用于在RViz中显示路径
void visPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub.publish(points);
}

// 根据当前时间计算在轨迹上的位置，用于在重新规划时确定起始点
Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t);
        return pos;
      }
    }
  }
  return pos;
}

// 根据当前时间计算在轨迹上的速度，用于在重新规划时确定起始速度
Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t);
        return Vel;
      }
    }
  }
  return Vel;
}
Vector3d getAcc(double t_cur) {
  double time = 0;
  Vector3d Acc = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time += 0.01;
      if (time > t_cur) {
        Acc = _trajGene->getAccPoly(_polyCoeff, i, t);
        return Acc;
      }
    }
  }
  return Acc;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");

  nh.param("planning/vel", _Vel, 1.0);
  nh.param("planning/acc", _Acc, 1.0);
  nh.param("planning/dev_order", _dev_order, 3);
  nh.param("planning/min_order", _min_order, 3);
  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
  nh.param("map/resolution", _resolution, 0.2);
  nh.param("map/x_size", _x_size, 50.0);
  nh.param("map/y_size", _y_size, 50.0);
  nh.param("map/z_size", _z_size, 5.0);
  nh.param("path/resolution", _path_resolution, 0.05);
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);

  _poly_num1D = 2 * _dev_order;

  _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);//核心，定时器回调

  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);
  _imu_sub = nh.subscribe("imu", 10, rcvImuCallback);

  _traj_pub =
      nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);

  // set the obstacle map
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
  _inv_resolution = 1.0 / _resolution;
  _max_x_id = (int)(_x_size * _inv_resolution);
  _max_y_id = (int)(_y_size * _inv_resolution);
  _max_z_id = (int)(_z_size * _inv_resolution);

  _astar_path_finder = new AstarPathFinder();
  _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper,
                                  _max_x_id, _max_y_id, _max_z_id);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}