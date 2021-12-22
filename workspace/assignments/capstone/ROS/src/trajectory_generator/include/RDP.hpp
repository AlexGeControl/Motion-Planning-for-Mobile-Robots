#ifndef QUAD_PLANNER_RDP_HPP_
#define QUAD_PLANNER_RDP_HPP_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <string>
#include <vector>

using std::vector;
//Given an array of points, "findMaximumDistance" calculates the GPS point which have largest distance from the line formed by first and last points in RDP algorithm. Returns the index of the point in the array and the distance.

namespace RDP {

const std::pair<int, double> FindMaxDistance(const std::vector<Eigen::Vector3d>& points) 
{
	// init result:
  	int index{0};  
  	double distMax{-1.0}; 

	// identify max distance:  
	const auto &first = points[0];
    const auto &last = points[points.size()-1];

  	const auto dp = last-first;
  	for(size_t i = 1; i < points.size()-1; ++i){
		// get point-to-line distance:
  		const auto dpCurr = points[i]-first;
  		const auto distCurr = dpCurr.cross(dp).norm() / dp.norm();

		// update: 
  		if (distCurr > distMax){
    		distMax = distCurr;
    		index = i;
  		}
	}

	// done
	return std::make_pair(index, distMax);
}

std::vector<size_t> DoRDP(
	const std::vector<size_t> &indices,
	const std::vector<Eigen::Vector3d> &points, 
	const double epsilon
) {
	std::vector<size_t> result;

	if (points.size() > 0u) {
		if (points.size() == 1u) {
			result.push_back(indices[0]);
		} else {
			const auto maxDistance = FindMaxDistance(points);
			
			if( maxDistance.second >= epsilon ){
				const int index = maxDistance.first;

				// first half:
				std::vector<size_t> xIndices(indices.cbegin(), indices.cbegin() + index + 1);
				std::vector<Eigen::Vector3d> xPoints(points.cbegin(), points.cbegin() + index + 1);

				// second half:
				std::vector<size_t> yIndices(indices.cbegin() + index, indices.cend());
				std::vector<Eigen::Vector3d> yPoints(points.cbegin() + index, points.cend()); 
				
				const auto resultX = DoRDP(xIndices, xPoints, epsilon);
				const auto resultY = DoRDP(yIndices, yPoints, epsilon);
				
				// get result:
				result.insert(result.end(), resultX.begin(), resultX.end());
				result.pop_back();
				result.insert(result.end(), resultY.begin(), resultY.end());
			}
			else {
				result.push_back(*indices.cbegin());
				result.push_back(*indices.crbegin());
			}
		}
	}

	return result;
}

} // namespace RDP

#endif // QUAD_PLANNER_PATH_FINDER_HPP_