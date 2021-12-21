#ifndef QUAD_PLANNER_TRAJECTORY_OPTIMIZER_HPP_
#define QUAD_PLANNER_TRAJECTORY_OPTIMIZER_HPP_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer();

  ~TrajectoryOptimizer();

  static constexpr double VelocityEpsilon{1e-5};

  /**
   * @brief get minimum num. of polynomial coeffs to satisfy [cOrder]th continuous constraints
   *
   * @param[in] cOrder continuity constraints, the target trajectory should be [cOrder]th continuous at intermediate waypoints
   *
   * @return minimum num of polynomial coeffs
   */
  static inline int GetNumCoeffs(const int cOrder) { return (cOrder + 1) << 1; }
  
  /**
   * @brief get partial factorial generated in derivative computing
   *
   * @param[in] N polynomial term power
   * @param[in] D derivative order
   *
   * @return partial factorial as N!/D!
   */
  static double GetFactorial(const int N, const int D);

  /**
   * @brief evaluate planned position at time t
   *
   * @param[in] coeffs polynomial coefficients
   * @param[in] N num. poly coeffs of trajectory segment
   * @param[in] k trajectory segment index
   * @param[in] t target timestamp
   *
   * @return planned position
   */
  static Eigen::Vector3d GetPos(const Eigen::MatrixXd &coeffs, const int N, const int k, const double t);

  /**
   * @brief evaluate planned velocity at time t
   *
   * @param[in] coeffs polynomial coefficients
   * @param[in] N num. poly coeffs of trajectory segment
   * @param[in] k trajectory segment index
   * @param[in] t target timestamp, scaled to [0.0, 1.0]
   *
   * @return planned velocity
   */
  static Eigen::Vector3d GetVel(const Eigen::MatrixXd &coeffs, const int N, const int k, const double t);

  enum class TimeAllocation {
    SegmentTrapezoidal,
    GlobalTrapezoidal
  };

  /**
   * @brief allocate traversal time for each trajectory segment
   *
   * @param[in] Path refined path from path finder, (K + 1)-by-3
   * @param[in] velLimit max. velocity
   * @param[in] accLimit max. acceleration
   * @param[in] strategy time allocation strategy, default to GlobalTrapezoidal
   *
   * @return allocated traversal times for each trajectory segment, K-by-1
   */
  static Eigen::VectorXd AllocateTimes(
    const Eigen::MatrixXd &Path, 
    const double velLimit,
    const double accLimit,
    const TimeAllocation strategy = TimeAllocation::GlobalTrapezoidal
  );

  enum class Solver {
    Numeric,
    Analytic
  };

  /**
   * @brief generate minimum snap trajectory
   *
   * @param[in] tOrder the L2-norm of [tOrder]th derivative of the target trajectory will be used as objective function
   * @param[in] cOrder continuity constraints, the target trajectory should be [cOrder]th continuous at intermediate waypoints
   * @param[in] Pos equality constraints, the target trajectory should pass all the waypoints, (K + 1)-by-D
   * @param[in] Vel equality constraints, boundary(start & goal) velocity specifications, 2-by-D
   * @param[in] Acc equality constraints, boundary(start & goal) acceleration specifications, 2-by-D
   * @param[in] Time pre-computed time allocations, K-by-1
   * @param[in] method solution method, defaults to Analytic
   *
   * @return polynomial coeffs of generated trajectory, K-by-(D * N)
   * @note the pre-assumption is no allocated segment time in Time is 0
   */
  static Eigen::MatrixXd GenerateTrajectory(
    const int tOrder,
    const int cOrder,                    
    const Eigen::MatrixXd &Pos,
    const Eigen::MatrixXd &Vel,
    const Eigen::MatrixXd &Acc,
    const Eigen::VectorXd &Time,
    Solver method = Solver::Analytic
  );

private:
  /**
   * @brief evaluate d-th order derivative of trajectory polynomial at time t
   *
   * @param[in] coeffs polynomial coefficients
   * @param[in] d derivative order
   * @param[in] t target timestamp
   *
   * @return result waypoint
   */
  static double EvaluatePoly(const Eigen::VectorXd &coeffs, const int d, const double t);

  /**
   * @brief allocate traversal time for each trajectory segment with segment-wise trapezoidal heuristics
   *
   * @param[in] Path refined path from path finder, (K + 1)-by-3
   * @param[in] velLimit max. velocity
   * @param[in] accLimit max. acceleration
   *
   * @return allocated traversal times for each trajectory segment, K-by-1
   */
  static Eigen::VectorXd DoSegmentTrapezoidalTimesAllocation(
    const Eigen::MatrixXd &Path,
    const double velLimit,
    const double accLimit
  );

  /**
   * @brief allocate traversal time for each trajectory segment with global trapezoidal heuristics
   *
   * @param[in] Path refined path from path finder, (K + 1)-by-3
   * @param[in] velLimit max. velocity
   * @param[in] accLimit max. acceleration
   *
   * @return allocated traversal times for each trajectory segment, K-by-1
   */
  static Eigen::VectorXd DoGlobalTrapezoidalTimesAllocation(
    const Eigen::MatrixXd &Path,
    const double velLimit,
    const double accLimit
  );

  /**
   * @brief generate minimum snap trajectory through numeric method with OSQP C++
   *
   * @param[in] tOrder the L2-norm of [tOrder]th derivative of the target trajectory will be used as objective function
   * @param[in] cOrder continuity constraints, the target trajectory should be [cOrder]th continuous at intermediate waypoints
   * @param[in] Pos equality constraints, the target trajectory should pass all the waypoints, (K + 1)-by-1
   * @param[in] Vel equality constraints, boundary(start & goal) velocity specifications, 2-by-1
   * @param[in] Acc equality constraints, boundary(start & goal) acceleration specifications, 2-by-1
   * @param[in] Time pre-computed time allocations, K-by-1
   *
   * @return polynomial coeffs of generated trajectory, K-by-N
   * @note the pre-assumption is no allocated segment time in Time is 0
   */
  static Eigen::MatrixXd DoTrajectoryGenerationNumerically(
    const int tOrder,       
    const int cOrder,    
    const Eigen::VectorXd &Pos,
    const Eigen::VectorXd &Vel,
    const Eigen::VectorXd &Acc,
    const Eigen::VectorXd &Time
  );

  /**
   * @brief generate minimum snap trajectory through analytic method with Eigen C++
   *
   * @param[in] tOrder the L2-norm of [tOrder]th derivative of the target trajectory will be used as objective function
   * @param[in] cOrder continuity constraints, the target trajectory should be [cOrder]th continuous at intermediate waypoints
   * @param[in] Pos equality constraints, the target trajectory should pass all the waypoints, (K + 1)-by-1
   * @param[in] Vel equality constraints, boundary(start & goal) velocity specifications, 2-by-1
   * @param[in] Acc equality constraints, boundary(start & goal) acceleration specifications, 2-by-1
   * @param[in] Time pre-computed time allocations, K-by-1
   *
   * @return polynomial coeffs of generated trajectory, K-by-N
   * @note the pre-assumption is no allocated segment time in Time is 0
   */
  static Eigen::MatrixXd DoTrajectoryGenerationAnalytically(
    const int tOrder,       
    const int cOrder,    
    const Eigen::VectorXd &Pos,
    const Eigen::VectorXd &Vel,
    const Eigen::VectorXd &Acc,
    const Eigen::VectorXd &Time
  );
};

#endif // QUAD_PLANNER_TRAJECTORY_OPTIMIZER_HPP_
