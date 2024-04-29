#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  
  auto delta = node1->coord -node2->coord;
  double manhattan = std::abs(delta.x()) + std::abs(delta.y()) + std::abs(delta.z());
  double euclidean = delta.norm();
  //对角线距离
  double dx = std::abs(delta.x()), dy = std::abs(delta.y()), dz = std::abs(delta.z());
  double diagonal = std::min({dx, dy, dz});
  double straight = dx + dy + dz - 2*diagonal;

  double h = straight;
  double p = 1.0 + (1.0 / 1000);
  h *= p;
  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;
  GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  while (!openSet.empty()) {
    auto lowestCostNode = openSet.begin();
    currentPtr = lowestCostNode -> second;
    openSet.erase(lowestCostNode);

    if(currentPtr->index == goalIdx){
      ros::Time time_2 = ros::Time::now();
      terminatePtr = currentPtr;
      ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
      return;
    }
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

    for(int i = 0; i < (int)neighborPtrSets.size(); i++){
      neighborPtr = neighborPtrSets[i];

      if(neighborPtr -> id == 0){
        double tentative_gScore = currentPtr->gScore + edgeCostSets[i];
        neighborPtr->cameFrom = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);

        openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));
        neighborPtr->id = 1; //加入开集
                
        continue;
      }
      else if(neighborPtr->id == 1){
        double tentative_gScore = currentPtr->gScore + edgeCostSets[i];
        if(tentative_gScore < neighborPtr->gScore){
        neighborPtr->cameFrom = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
        // openSet.erase(neighborPtr->nodeMapIt);//删除旧节点
        // neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));

        }
        continue;
      }
      else{
        continue;
      }
    }
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  auto temPtr = terminatePtr;
  while(temPtr->cameFrom != NULL){
    gridPath.push_back(temPtr);
    temPtr = temPtr->cameFrom;
  }
  for(auto ptr:gridPath)
    path.push_back(ptr->coord);
  
  reverse(path.begin(), path.end());
  ROS_INFO("This is a getPath info messgae.");
  return path;
}

//计算一个点到由两个端点定义的直线的垂直距离
double perpendicularDistance(const Vector3d &point, const Vector3d &lineStart, const Vector3d &lineEnd) {
    Vector3d lineVec = lineEnd - lineStart;
    Vector3d pointVec = point - lineStart;
    Vector3d lineVecNorm = lineVec.normalized();
    double distance = (pointVec - pointVec.dot(lineVecNorm) * lineVecNorm).norm();
    return distance;
}

//递归函数，实现 RDP 算法，使用 perpendicularDistance 来找到距离直线最远的点，并根据这个距离决定是否继续递归简化路径
void DouglasPeucker(const vector<Vector3d> &pointList, double epsilon, vector<Vector3d> &out) {
    if (pointList.size() < 2) {
        throw std::runtime_error("Not enough points to simplify");
    }

    // Find the point with the maximum distance
    double dmax = 0.0;
    size_t index = 0;
    for (size_t i = 1; i < pointList.size() - 1; i++) {
        double d = perpendicularDistance(pointList[i], pointList.front(), pointList.back());
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        vector<Vector3d> recResults1, recResults2;
        vector<Vector3d> firstPart(pointList.begin(), pointList.begin() + index + 1);
        vector<Vector3d> secondPart(pointList.begin() + index, pointList.end());
        DouglasPeucker(firstPart, epsilon, recResults1);
        DouglasPeucker(secondPart, epsilon, recResults2);

        // Build the result list
        out.reserve(recResults1.size() + recResults2.size() - 1);
        out.insert(out.end(), recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
    } else {
        // Just return start and end points
        out.push_back(pointList.front());
        out.push_back(pointList.back());
    }
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/
  DouglasPeucker(path, path_resolution, subPath);
  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
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

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/
  // PolyCoeff(m,3 * 6)
  // | c0_x c1_x c2_x c3_x c4_x c5_x c0_y c1_y c2_y c3_y c4_y c5_y c0_z c1_z c2_z c3_z c4_z c5_z|
  // .......第n行就是第n段轨迹的系数
  // | c0_x c1_x c2_x c3_x c4_x c5_x c0_y c1_y c2_y c3_y c4_y c5_y c0_z c1_z c2_z c3_z c4_z c5_z|
  //路径是index,用isFree()和isOccupied()

  // 遍历每一段轨迹
    for (int i = 0; i < polyCoeff.rows(); ++i) {
        // 获取当前段的系数和时间
        double t_segment = time(i); // 当前段的持续时间
        // MatrixXd coeffs = polyCoeff.row(i).reshaped(6, 3); // 重新塑形为6行3列，每列对应x,y,z
        MatrixXd coeffs(6, 3); // 创建一个6行3列的矩阵
        coeffs.col(0) = polyCoeff.block(i, 0, 1, 6).transpose(); // X 维度系数
        coeffs.col(1) = polyCoeff.block(i, 6, 1, 6).transpose(); // Y 维度系数
        coeffs.col(2) = polyCoeff.block(i, 12, 1, 6).transpose(); // Z 维度系数

        // 采样当前段的轨迹以检查安全性
        double t_step = 0.2; // 每0.1秒采样一次
        for (double t = 0; t <= t_segment; t += t_step) {
            Vector3d point; // 存储当前采样点的位置
            // 对每个维度（x, y, z）计算轨迹上的点
            for (int dim = 0; dim < 3; ++dim) {
                point(dim) = coeffs(0, dim) + coeffs(1, dim) * t + coeffs(2, dim) * pow(t, 2) +
                             coeffs(3, dim) * pow(t, 3) + coeffs(4, dim) * pow(t, 4) + coeffs(5, dim) * pow(t, 5);
            }
            Vector3i point_index = coord2gridIndex(point);
            // 检查该点是否在被障碍物占据的区域
            if (isOccupied(point_index)) {
                unsafe_segment = i; // 记录不安全的段
                ROS_INFO("\033[35mUnsafe segment(%d)\033[0m",i);
                return unsafe_segment; // 一旦发现不安全即返回
            }
        }
    }
  

  return unsafe_segment;
}