#include "trajectory_generator_waypoint.h"

#include "osqp++.h"

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

/**
  * @brief get partial factorial generated in derivative computing
  *
  * @param[in] N polynomial term power
  * @param[in] D derivative order
  *
  * @return partial factorial as N!/D!
  */
double TrajectoryGeneratorWaypoint::GetFactorial(const int N, const int D) {
    double result{1.0};

    for (int i = 0; i < D; ++i) {
        result *= (N - i);
    }

    return result;
}

/**
  * @brief evaluate trajectory polynomial
  *
  * @param[in] N polynomial term power
  * @param[in] D derivative order
  *
  * @return result waypoint
  */
double TrajectoryGeneratorWaypoint::EvaluatePoly(const Eigen::VectorXd &coeffs, const double t) {
    const int N = coeffs.size();

    Eigen::VectorXd powers = Eigen::VectorXd::Zero(N);
    for (int n = 0; n < N; ++n) {
        powers(n) = ((n == 0) ? 1.0 : t*powers(n - 1));
    }

    return coeffs.dot(powers);
}

/**
 * @brief generate minimum snap trajectory through numeric method with OSQP C++
 *
 * @param[in] tOrder the L2-norm of [tOrder]th derivative of the target trajectory will be used as objective function
 * @param[in] cOrder continuity constraints, the target trajectory should be [cOrder]th continuous at intermediate waypoints
 * @param[in] Pos equality constraints, the target trajectory should pass all the waypoints, (K + 1)-by-D
 * @param[in] Vel equality constraints, boundary(start & goal) velocity specifications, 2-by-D
 * @param[in] Acc equality constraints, boundary(start & goal) acceleration specifications, 2-by-D
 * @param[in] Time pre-computed time allocations, K-by-1
 *
 * @return polynomial coeffs of generated trajectory, K-by-(D * N)
 * @note the pre-assumption is no allocated segment time in Time is 0
 */
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int tOrder,
    const int cOrder,                    
    const Eigen::MatrixXd &Pos,
    const Eigen::MatrixXd &Vel,
    const Eigen::MatrixXd &Acc,
    const Eigen::VectorXd &Time
) {
    const int N = GetNumCoeffs(cOrder);
    const int K = Time.size(); 

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(K, 3 * N);
    
    for (int i = 0; i < Pos.cols(); ++i) {
        result.block(0, i*N, K, N) = PolyQPGenerationNumeric(
            // minimum snap as objective
            tOrder,
            cOrder,
            Pos.col(i),
            Vel.col(i),
            Acc.col(i),
            Time
        );
    }

    return result;
}

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
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGenerationNumeric(
    const int tOrder,       
    const int cOrder,    
    const Eigen::VectorXd &Pos,
    const Eigen::VectorXd &Vel,
    const Eigen::VectorXd &Acc,
    const Eigen::VectorXd &Time
) {
    // num. of polynomial coeffs:
    const auto N = GetNumCoeffs(cOrder);
    // num. of trajectory segments:
    const auto K = Time.size();
    // dim of flattened output:
    const auto D = K * N;
    // num. of inequality constraints:
    const auto actualOrder = std::max(2, cOrder);
    const auto C = (
        // 1. boundary value equality constraints:
        ((actualOrder + 1) << 1) + 
        // 2. intermediate waypoint passing equality constraints:
        (K - 1) + 
        // 3. intermediate waypoint continuity constaints:
        (K - 1) * (cOrder + 1)
    );

    //
    // init output:
    //
    //     the trajectory segment k is defined by result(k, 0) + t*(result(k, 1) + ...)
    // 
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(K, N);

    //
    // problem definition:
    // 
    //     min: 0.5 * x'Px + q'x
    //     s.t. l <= Ax <= u
    // 
    // osqp-cpp interface
    //
    //     P: objective_matrix
    //     q: objective_vector
    //     A: constraint_matrix
    //     l: lower_bounds
    //     u: upper_bounds
    // 

    // 
    // define objective matrix P:
    //
    // 1. init:
    Eigen::SparseMatrix<double> P(D, D);
    std::vector<Eigen::Triplet<double>> PTriplets;
    
    // 2. cache results for objective matrix construction:
    
    // 2.a time power
    Eigen::VectorXd PTimePower = Eigen::VectorXd::Ones(K);
    for (int c = 1; c < tOrder; ++c) {
        PTimePower = PTimePower.cwiseProduct(Time);
    }
    
    // 2.b factorial
    std::map<int, double> PFactorial;
    for (int n = tOrder; n < N; ++n) {
        PFactorial.insert(
            {n, GetFactorial(n, tOrder)}
        );
    }

    // 3. populate PTriplets:
    for (int k = 0; k < K; ++k) {
        const auto currentSegmentIdxOffset = k * N;

        for (int m = tOrder; m < N; ++m) {
            for (int n = tOrder; n < N; ++n) {
                PTriplets.emplace_back(
                    currentSegmentIdxOffset + m,
                    currentSegmentIdxOffset + n,
                    Time(k)*PFactorial[m]*PFactorial[n]/(
                        (m + n - (tOrder << 1) + 1) *
                        PTimePower(k) * PTimePower(k)
                    )
                );
            }
        }
    }

    // 4. populate P:
    P.setFromTriplets(std::begin(PTriplets),std::end(PTriplets));

    //
    // define constraint matrix A:
    //
    // 1. init:
    Eigen::SparseMatrix<double> A(C, D);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(C);
    std::vector<Eigen::Triplet<double>> ATriplets;

    // 2. cache results for constraint matrix construction:
    std::map<int, Eigen::VectorXd> ATimePower;
    std::map<int, double> AFactorial;

    // 2.a init:
    ATimePower.insert(
        {0, Eigen::VectorXd::Ones(K)}
    );
    
    auto GetAFactorialKey = [&](const int n, const int D) {
        return D * N + n;
    };
    for (int n = 0; n < N; ++n) {
        AFactorial.insert(
            {GetAFactorialKey(n, 0), 1.0}
        );
    }

    for (int c = 1; c <= cOrder; ++c) {
        // 2.b time power
        ATimePower.insert(
            {c, ATimePower[c - 1].cwiseProduct(Time)}
        );

        // 2.c factorial
        for (int n = c; n < N; ++n) {
            AFactorial.insert(
                {GetAFactorialKey(n, c), (n - c + 1)*AFactorial[GetAFactorialKey(n, c - 1)]}
            );
        }
    };

    // 3. populate ATriplets:
    {
        int currentConstraintIdxOffset{0};

        //
        // 3.1 boundary value equality constraints:
        //

        // init:
        Eigen::MatrixXd boundaryValues = Eigen::MatrixXd::Zero(2, actualOrder + 1);
        boundaryValues(0, 0) = Pos(0);
        boundaryValues(0, 1) = Vel(0);
        boundaryValues(0, 2) = Acc(0);
        boundaryValues(1, 0) = Pos(K);
        boundaryValues(1, 1) = Vel(1);
        boundaryValues(1, 2) = Acc(1);

        // populate constraints:
        for (int c = 0; c <= actualOrder; ++c) {
            // start waypoint:
            ATriplets.emplace_back(
                currentConstraintIdxOffset, c, AFactorial[GetAFactorialKey(c, c)]/ATimePower[c](0)
            );
            b(currentConstraintIdxOffset) = boundaryValues(0, c);
            
            ++currentConstraintIdxOffset;

            // end waypoint:
            for (int n = c; n < N; ++n) {
                ATriplets.emplace_back(
                    currentConstraintIdxOffset, (K - 1)*N + n, AFactorial[GetAFactorialKey(n, c)]/ATimePower[c](K - 1)
                );
            }
            b(currentConstraintIdxOffset) = boundaryValues(1, c);

            ++currentConstraintIdxOffset;
        }

        //
        // 3.2 intermediate waypoint passing equality constraints:
        //
        for (int k = 1; k < K; ++k) {
            ATriplets.emplace_back(
                currentConstraintIdxOffset, k*N, 1.0
            );
            b(currentConstraintIdxOffset) = Pos(k);

            ++currentConstraintIdxOffset;
        }

        //
        // 3.3 intermediate waypoint continuity constaints:
        //
        for (int c = 0; c <= cOrder; ++c) {
            for (int k = 1; k < K; ++k) {
                // the end of previous trajectory segment:
                for (int n = c; n < N; ++n) {
                    ATriplets.emplace_back(
                        currentConstraintIdxOffset, 
                        (k - 1) * N + n,
                        AFactorial[GetAFactorialKey(n, c)]/ATimePower[c](k - 1)
                    );
                }

                // should equal to the start of current trajectory segment:
                ATriplets.emplace_back(
                    currentConstraintIdxOffset,
                    k * N + c,
                    -AFactorial[GetAFactorialKey(c, c)]/ATimePower[c](k) 
                );

                // set next constraint:
                ++currentConstraintIdxOffset;
            }
        }
    }

    // 4. populate A:
    A.setFromTriplets(std::begin(ATriplets),std::end(ATriplets));

    //
    // solve with OSQP c++
    //
    osqp::OsqpInstance instance;
    instance.objective_matrix = P;
    instance.objective_vector = Eigen::VectorXd::Zero(D);

    instance.constraint_matrix = A;
    instance.lower_bounds = instance.upper_bounds = b;

    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;

    // init solver:
    if (solver.Init(instance, settings).ok()) {
        // solve.
        const auto exitCode = solver.Solve();

        if (exitCode == osqp::OsqpExitCode::kOptimal) {
            // get optimal solution
            const auto optimalObject = solver.objective_value();
            const auto optimalCoeffs = solver.primal_solution();

            //
            // format output:
            //
            for (int k = 0; k < K; ++k) {
                result.row(k) = optimalCoeffs.segment(k * N, N);
            }

            ROS_INFO("[Minimum Snap, Numeric]: Optimal objective is %.2f.", optimalObject);
        } else {
            // defaults to Eigen::MatrixXd::Zero():
            ROS_WARN("[Minimum Snap, Numeric]: Failed to find the optimal solution.");
        }
    } else {
        // defaults to Eigen::MatrixXd::Zero():
        ROS_WARN("[Minimum Snap, Numeric]: Failed to init OSQP solver.");
    }

    // done:
    return result;
}
