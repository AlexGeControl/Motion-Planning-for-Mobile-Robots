#ifndef QUAD_PLANNER_PATH_FINDER_HPP_
#define QUAD_PLANNER_PATH_FINDER_HPP_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

class PathFinder
{	
private:
	uint8_t * data;
	GridNodePtr *** GridNodeMap;
	Eigen::Vector3i goalIdx;
	int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
	int GLXYZ_SIZE, GLYZ_SIZE;

	double resolution, inv_resolution;
	double gl_xl, gl_yl, gl_zl;
	double gl_xu, gl_yu, gl_zu;

	GridNodePtr terminatePtr;
	std::multimap<double, GridNodePtr> openSet;

	inline bool isOccupied(const Eigen::Vector3i &index) const {
		return isOccupied(index(0), index(1), index(2));
	}

	inline bool isFree(const Eigen::Vector3i &index) const {
		return isFree(index(0), index(1), index(2));
	}

	inline bool isOccupied(
		const int idx_x, 
		const int idx_y,
		const int idx_z
	) const {
		return (
			idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
			(data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1)
		) || (
			idx_x < 0 || idx_x >= GLX_SIZE || idx_y < 0 || idx_y >= GLY_SIZE
		);
	}

	inline bool isFree(
		const int idx_x, 
		const int idx_y,
		const int idx_z
	) const {
		return (
			idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
			(data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1)
		);
	}

	double getHeu(
		GridNodePtr sourcePtr, 
		GridNodePtr targetPtr, 
		GridNodePtr currPtr 
	);
	void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

	Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
	Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

public:
	static constexpr int NullIndex{-1};

	PathFinder(){};
	~PathFinder(){};
	void FindPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	void resetGrid(GridNodePtr ptr);
	void resetUsedGrids();

	void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
	void setObs(const double coord_x, const double coord_y, const double coord_z);

	Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
	std::vector<Eigen::Vector3d> GetPath();
	std::vector<Eigen::Vector3d> getVisitedNodes();
	/**
	  * @brief simplify waypoints from path finding using RDP
	  *
	  * @param[in] path raw waypoints from path finder
	  * @param[in] path_resolution spatial resolution of simplified waypoints
	  *
	  * @return vector of index of selected critical waypoints
	  */
	std::vector<size_t> SimplifyPath(
		const std::vector<Eigen::Vector3d> &path, 
		const double path_resolution
	);

	/**
	  * @brief refine waypoints for segment with collision
	  *
	  * @param[in] input raw waypoint indices
	  * @param[in] segment_index first index of segment with collsion
	  *
	  * @return refined waypoint indices
	  */
	std::vector<size_t> RefinePath(
		const std::vector<size_t> &input, 
		size_t segment_index
	);

	/**
	  * @brief detect collision on planned trajectory
	  *
	  * @param[in] coeffs polynomial coefficients of each trajectory segment
	  * @param[in] time allocated time for each trajectory segment
	  * @param[in] timeResolution temporal sampling resolution for collision check
	  *
	  * @return index of first unsafe trajectory segment
	  */
	template <typename FuncGetPos>
	int DetectCollision(
		const Eigen::MatrixXd &polyCoeff, 
  		const Eigen::VectorXd &time,
		double timeResolution,
		FuncGetPos GetPos
	) {
		const size_t K = time.size();
		const size_t N = polyCoeff.cols() / 3;

		for (size_t k = 0; k < K; ++k) {
			for (double t = 0.0; t <= time(k); t += timeResolution) {
				const Eigen::Vector3d pos = GetPos(polyCoeff, N, k, t);
				const Eigen::Vector3i idx = coord2gridIndex(pos);

				const int idxX = idx(0);
				const int idxY = idx(1);
				const int idxZ = idx(2);
				if (!isFree(idxX, idxY, idxZ)) {
					return k;
				}
			}
		}

		return PathFinder::NullIndex;
	}
};

#endif // QUAD_PLANNER_PATH_FINDER_HPP_