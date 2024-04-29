#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial 这里d_order=3
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  const int waypointNum = Path.rows();
  // const int pieceNum = Time.size();
  const int pieceNum = m;
  if(waypointNum != m+1){
    ROS_ERROR("num of time (%d) != num of trajectory(%d)", m, pieceNum);
  }else{
    ROS_INFO("num of time (%d) == num of trajectory(%d)", m, pieceNum);
  }
  const Eigen::Vector3d initialPos = Path.row(0);
  const Eigen::Vector3d initialVel = Vel.row(0);
  const Eigen::Vector3d initialAcc = Acc.row(0);
  const Eigen::Vector3d terminalPos = Path.row(Path.rows()-1);
  const Eigen::Vector3d terminalVel = Vel.row(1);
  const Eigen::Vector3d terminalAcc = Acc.row(1);
  Eigen::MatrixXd intermediatePositions;

  std::stringstream s3;
  s3 << "waypoint:\n" << Path;
  ROS_INFO("\033[35m%s\033[0m", s3.str().c_str());

  if (Path.rows() > 2) {
  // ROS_INFO("\033[35mpath.rows()=%d,path.cols=%d\033[0m",Path.rows(),Path.cols());
  // intermediatePositions = (Path.block(1, 0, Path.rows() - 2, Path.cols())).transpose();
  intermediatePositions = Path.block(1, 0, Path.rows() - 2, Path.cols()).transpose();


  // std::stringstream ss;
  // ss << "Intermediate Positions:\n" << intermediatePositions;
  // ROS_INFO("\033[35m%s\033[0m", ss.str().c_str());

  } else {
    // Handle the case where there are no intermediate points
    ROS_INFO("only start ptr and end ptr");
    // Eigen::Matrix3Xd intermediatePositions;
  }
  Eigen::VectorXd timeAllocationVector = Time;

  std::stringstream ss2;
  ss2 << "Time Allocation Vector: [";
  for (int i = 0; i < timeAllocationVector.size(); ++i) {
      ss2 << timeAllocationVector(i);
      if (i != timeAllocationVector.size() - 1) {
          ss2 << ", ";
      }
  }
  ss2 << "]";

  // 使用ROS_INFO打印字符串
  ROS_INFO("%s", ss2.str().c_str());

  Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

  //这里直接把minium_jerk的作业用到这里
  // ------------------------ Put your solution below ------------------------
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6*pieceNum, 6*pieceNum);
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6*pieceNum, 3);

  Eigen::MatrixXd F_0(3,6);
  F_0 << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 2, 0, 0, 0;

  M.block(0,0,3,6) = F_0;
  b.block(0,0,3,3) << initialPos(0), initialPos(1), initialPos(2),
                      initialVel(0), initialVel(1), initialVel(2),
                      initialAcc(0), initialAcc(1), initialAcc(2);
  
  double t(timeAllocationVector(pieceNum-1));
  Eigen::MatrixXd E_M(3,6);
  E_M << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5),
          0, 1, 2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
          0, 0, 2, 6*t, 12*pow(t,2), 20*pow(t,3);
      
  M.block(6*pieceNum-3, 6*(pieceNum-1), 3, 6) = E_M;
  b.block(6*pieceNum-3, 0, 3, 3) << terminalPos(0), terminalPos(1), terminalPos(2),
                                    terminalVel(0), terminalVel(1), terminalVel(2),
                                    terminalAcc(0), terminalAcc(1), terminalAcc(2);
  // ROS_INFO("\033[35m============This is a test message2==============\033[0m");


  for(int i=1; i<pieceNum; i++){
      double t(timeAllocationVector(i-1));
      Eigen::MatrixXd F_i(6,6), E_i(6,6);
      Eigen::Vector3d D_i(intermediatePositions.transpose().row(i-1));
      E_i << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5),
              1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5),
              0, 1, 2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
              0, 0, 2, 6*t, 12*pow(t,2), 20*pow(t,3),
              0, 0, 0, 6, 24*t, 60*pow(t,2),
              0, 0, 0, 0, 24, 120*t;
  // ROS_INFO("\033[35m============This is a test message==============\033[0m");

      F_i << 0, 0, 0, 0, 0, 0, 
            -1, 0, 0, 0, 0, 0,
            0, -1, 0, 0, 0, 0,
            0, 0, -2, 0, 0, 0, 
            0, 0, 0, -6, 0, 0, 
            0, 0, 0, 0, -24, 0;
      
      int j = 6 * (i-1);
      M.block(3+6*(i-1), j+6, 6, 6) = F_i;
      M.block(3+6*(i-1), j, 6, 6) = E_i;
      b.block(3+6*(i-1), 0, 6, 3) << D_i(0), D_i(1), D_i(2),
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0;
          
  }
  // ROS_INFO("\033[35m============This is a test message3==============\033[0m");

  clock_t time_stt = clock();

  coefficientMatrix = M.lu().solve(b);
  // coefficientMatrix = M.inverse() * b;

  // std::cout << "Time cost = " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
  ROS_INFO("caculate minimum_jerk cost %d ms", 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC);

  // ------------------------ Put your solution above ------------------------

  //FROM coefficientMatrix(6 * m, 3) TO PolyCoeff(m,3 * 6)
  // 遍历每个轨迹段
  for (int i = 0; i < pieceNum; ++i) {
    // 遍历每个系数
    for (int j = 0; j < p_num1d; ++j) {
        // 将coefficientMatrix中的系数转移到PolyCoeff中，按维度顺序排列
        PolyCoeff(i, j) = coefficientMatrix(i * 6 + j, 0); // X 维度
        PolyCoeff(i, j + p_num1d) = coefficientMatrix(i * 6 + j, 1); // Y 维度
        PolyCoeff(i, j + 2 * p_num1d) = coefficientMatrix(i * 6 + j, 2); // Z 维度
    }
}
  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}