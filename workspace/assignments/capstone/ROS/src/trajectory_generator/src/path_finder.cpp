#include "RDP.hpp"
#include "path_finder.hpp"

using namespace std;
using namespace Eigen;

void PathFinder::initGridMap(
  double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, 
  int max_x_id, int max_y_id, int max_z_id
) {
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

void PathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void PathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void PathFinder::setObs(const double coord_x, const double coord_y,
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

vector<Vector3d> PathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        if (GridNodeMap[i][j][k]->id == -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d PathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i PathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
         min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
         min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d PathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline void PathFinder::AstarGetSucc(
  GridNodePtr currentPtr,
  vector<GridNodePtr> &neighborPtrSets,
  vector<double> &edgeCostSets
) {
  // init output buffers:
  neighborPtrSets.clear();
  edgeCostSets.clear();

  // iterate over all adjacent grids:
  for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
              if (
                  // do not process current node
                  !(
                      (0 == dx) && (0 == dy) && (0 == dz)
                  )
              ) {
                  const auto& current_index = currentPtr->index;

                  const auto neighbor_index = Eigen::Vector3i(
                      current_index(0) + dx,
                      current_index(1) + dy,
                      current_index(2) + dz
                  );

                  if (
                      // a. do not process node outside map boundaries:
                      (
                          (neighbor_index(0) >= 0 && neighbor_index(0) < GLX_SIZE) && 
                          (neighbor_index(1) >= 0 && neighbor_index(1) < GLY_SIZE) && 
                          (neighbor_index(2) >= 0 && neighbor_index(2) < GLZ_SIZE)
                      ) && 
                      // b. only evaluate free grid:
                      (
                          isFree(neighbor_index)
                      ) && 
                      // c. not in close set:
                      (
                          GridNodeMap[neighbor_index(0)][neighbor_index(1)][neighbor_index(2)]->id != -1
                      )
                  ) {
                      // update candidate neighbor node:
                      auto& neighbor_node = GridNodeMap[neighbor_index(0)][neighbor_index(1)][neighbor_index(2)];

                      neighbor_node->dir = Eigen::Vector3i(dx, dy, dz);
                      neighbor_node->cameFrom = currentPtr;

                      neighborPtrSets.push_back(neighbor_node);
                      edgeCostSets.push_back(resolution*(neighbor_index - current_index).norm());
                  }
              }
          }
      }
  }
}

double PathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  /*
      STEP 1: finish the PathFinder::getHeu , which is the heuristic function 
          a. choose possible heuristic function you want
              Manhattan
              Euclidean
              Diagonal
              or 0 (Dijkstra)
          b. Remember tie_breaker learned in lecture, add it here ?
  */

  return (node2->coord - node1->coord).norm();
}

void PathFinder::FindPath(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();    

  //index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx   = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  //position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt   = gridIndex2coord(end_idx);

  //Initialize the pointers of struct GridNode which represent start node and goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();

  // currentPtr represents the node with lowest f(n) in the open_list
  double currentCost = 0.0;
  GridNodePtr currentPtr  = NULL;
  GridNodePtr neighborPtr = NULL;

  //put start node in open set
  startPtr -> gScore = 0;
  startPtr -> fScore = getHeu(startPtr, endPtr);   
  /*
      STEP 1: finish the PathFinder::getHeu , which is the heuristic function
  */
  startPtr -> id = 1; 
  startPtr -> coord = start_pt;
  startPtr -> cameFrom = NULL;
  openSet.insert( make_pair(startPtr -> fScore, startPtr) );
  /*
      STEP 2 :  some else preparatory works which should be done before while loop
  */
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  // this is the main loop
  while ( !openSet.empty() ){
      /*
          STEP 3: Remove the node with lowest cost function from open set to closed set
      */
      currentPtr = openSet.begin()->second;
      currentCost = currentPtr->gScore;
      
      // add into close set:
      currentPtr->id = -1;
      openSet.erase(openSet.begin());

      // if the current node is the goal 
      if( currentPtr->index == goalIdx ){
          ros::Time time_2 = ros::Time::now();
          terminatePtr = currentPtr;
          ROS_WARN(
            "[PathFinder::FindPath]: SUCCEEDED -- time consumption is %.3f ms, path length is %.2f meters", 
            1000*(time_2 - time_1).toSec(), 
            currentPtr->gScore * resolution 
          );            
          return;
      }

      /*
          STEP 4: get the successors
      */
      AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     

      /*
          STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
      */         
      for(int i = 0; i < (int)neighborPtrSets.size(); i++){
          /*
              Judge if the neigbors have been expanded:
                  neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
                  neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set       
          */
          neighborPtr = neighborPtrSets.at(i);

          auto gScore = currentCost + edgeCostSets.at(i);
          auto fScore = gScore + getHeu(neighborPtr, endPtr);
          // CASE 1: discover a new node, which is not in the closed set and open set
          if( 0 == neighborPtr-> id ){ 
              /*
                  STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
              */
              neighborPtr->gScore = gScore;
              neighborPtr->fScore = fScore;

              neighborPtr->id = 1;
              openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) );
          }
          // CASE 2: this node is in open set and need to judge if it needs to update
          else if( 1 == neighborPtr-> id ){ 
              /*
                  STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it      
              */
              auto neighbors_to_be_updated = openSet.equal_range(neighborPtr->fScore);
              for (auto neighbor_itr = neighbors_to_be_updated.first; neighbors_to_be_updated.second != neighbor_itr; ++neighbor_itr) {
                  auto& neighbor_to_be_updated = neighbor_itr->second;
                  if (
                      (
                          (neighbor_to_be_updated->index(0) == neighborPtr->index(0)) &&
                          (neighbor_to_be_updated->index(1) == neighborPtr->index(1)) &&
                          (neighbor_to_be_updated->index(2) == neighborPtr->index(2))
                      ) && (
                          fScore < neighborPtr->fScore
                      )
                  ) {
                      openSet.erase(neighbor_itr);

                      neighborPtr->gScore = gScore;
                      neighborPtr->fScore = fScore;

                      openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) );

                      break;
                  }
              }
          }
          // CASE 3: this node is in closed set
          else{
              // skip:
              continue;
          }
      }     
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if((time_2 - time_1).toSec() > 0.1)
      ROS_WARN("[PathFinder::FindPath]: FAILED -- time consumption is %.3f ms.", 1000*(time_2 - time_1).toSec());
}

vector<Vector3d> PathFinder::GetPath() 
{
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;
  
  auto currentPtr = terminatePtr;
  while (NULL != currentPtr) {
      gridPath.push_back(currentPtr);
      currentPtr = currentPtr->cameFrom;
  }

  for (auto it = gridPath.rbegin(); it != gridPath.rend(); ++it) {
      const auto& curr_coord = (*it)->coord;
      path.push_back(curr_coord);
  }

  return path;
}

std::vector<size_t> PathFinder::SimplifyPath(
  const std::vector<Eigen::Vector3d> &waypoints,
  double path_resolution
) {
  std::vector<size_t> indices(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    indices[i] = i;
  }

  return RDP::DoRDP(indices, waypoints, path_resolution);
}