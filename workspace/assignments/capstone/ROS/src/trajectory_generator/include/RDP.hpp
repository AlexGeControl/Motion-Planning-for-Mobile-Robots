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

std::vector<Eigen::Vector3d> DoRDP(
	const std::vector<Eigen::Vector3d>& points, 
	const double epsilon
) {
	std::vector<Eigen::Vector3d> result;

	if (points.size() > 0u) {
		if (points.size() == 1u) {
			result.push_back(*points.cbegin());
		} else {
			const auto maxDistance = FindMaxDistance(points);
			
			if( maxDistance.second >= epsilon ){
				const int index = maxDistance.first;

				// first half:
				std::vector<Eigen::Vector3d> firstHalf(points.cbegin(), points.cbegin() + index + 1);
				// second half:
				std::vector<Eigen::Vector3d> secondHalf(points.cbegin() + index, points.cend()); 
				
				const auto resultFirstHalf = DoRDP(firstHalf, epsilon);
				const auto resultSecondHalf = DoRDP(secondHalf, epsilon);
				
				// get result:
				result.insert(result.end(), resultFirstHalf.begin(), resultFirstHalf.end());
				result.pop_back();
				result.insert(result.end(), resultSecondHalf.begin(), resultSecondHalf.end());

				ROS_WARN(
					"[RDP]: max. dist is %.2f, shrink from %d to %d", 
					maxDistance.second, 
					points.size(), result.size()
				);
			}
			else {
				result.push_back(*points.cbegin());
				result.push_back(*points.crbegin());
			}
		}
	}

	return result;
}

} // namespace RDP

#endif // QUAD_PLANNER_PATH_FINDER_HPP_