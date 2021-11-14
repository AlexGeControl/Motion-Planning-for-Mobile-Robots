#ifndef _GRID_SEARCHER_H_
#define _GRID_SEARCHER_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

class RRTstarPreparatory
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr *** GridNodeMap;

		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;	

		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

	public:
		RRTstarPreparatory(){};
		~RRTstarPreparatory(){};

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);
		bool isObsFree(const double coord_x, const double coord_y, const double coord_z);
		
		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
};

#endif