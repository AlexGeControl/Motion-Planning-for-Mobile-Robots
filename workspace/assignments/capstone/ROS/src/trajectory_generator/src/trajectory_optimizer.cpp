#include "trajectory_optimizer.hpp"

#include <osqp++.h>

#include <ros/ros.h>

#include <chrono>
#include <string>
#include <vector>
#include <map>

TrajectoryOptimizer::TrajectoryOptimizer(){}
TrajectoryOptimizer::~TrajectoryOptimizer(){}

/**
  * @brief get partial factorial generated in derivative computing
  *
  * @param[in] N polynomial term power
  * @param[in] D derivative order
  *
  * @return partial factorial as N!/D!
  */
double TrajectoryOptimizer::GetFactorial(const int N, const int D) {
    double result{1.0};

    for (int i = 0; i < D; ++i) {
        result *= (N - i);
    }

    return result;
}

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
Eigen::Vector3d TrajectoryOptimizer::GetPos(const Eigen::MatrixXd &coeffs, const int N, const int k, const double t) {     
    Eigen::Vector3d pos;

    for ( int dim = 0; dim < 3; dim++ )
    {
        const Eigen::VectorXd &segmentCoeffs = (coeffs.row(k)).segment( dim * N, N );
        pos(dim) = EvaluatePoly(segmentCoeffs, 0, t);
    }

    return pos; 
}

/**
  * @brief evaluate planned velocity at time t
  *
  * @param[in] coeffs polynomial coefficients
  * @param[in] N num. poly coeffs of trajectory segment
  * @param[in] k trajectory segment index
  * @param[in] t target timestamp
  *
  * @return planned velocity
  */
Eigen::Vector3d TrajectoryOptimizer::GetVel(const Eigen::MatrixXd &coeffs, const int N, const int k, const double t) { 
    Eigen::Vector3d vel;

    for ( int dim = 0; dim < 3; dim++ )
    {
        const Eigen::VectorXd &segmentCoeffs = (coeffs.row(k)).segment( dim * N, N );
        vel(dim) = EvaluatePoly(segmentCoeffs, 1, t);
    }

    return vel; 
}

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
Eigen::VectorXd TrajectoryOptimizer::AllocateTimes(
  const Eigen::MatrixXd &Path, 
  const double velLimit,
  const double accLimit,
  const TrajectoryOptimizer::TimeAllocation strategy
) {
    const int K = (Path.rows() - 1);
    
    Eigen::VectorXd time = Eigen::VectorXd::Zero(K);

    switch (strategy) {
        case TrajectoryOptimizer::TimeAllocation::SegmentTrapezoidal:
            return DoSegmentTrapezoidalTimesAllocation(Path, velLimit, accLimit);
            break;
        case TrajectoryOptimizer::TimeAllocation::GlobalTrapezoidal:
            return DoGlobalTrapezoidalTimesAllocation(Path, velLimit, accLimit);
            break;
        default:
            break;
    }

    return time;
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
 * @param[in] method solution method, defaults to Analytic
 *
 * @return polynomial coeffs of generated trajectory, K-by-(D * N)
 * @note the pre-assumption is no allocated segment time in Time is 0
 */
Eigen::MatrixXd TrajectoryOptimizer::GenerateTrajectory(
    const int tOrder,
    const int cOrder,                    
    const Eigen::MatrixXd &Pos,
    const Eigen::MatrixXd &Vel,
    const Eigen::MatrixXd &Acc,
    const Eigen::VectorXd &Time,
    const TrajectoryOptimizer::Solver method
) {
    const int N = GetNumCoeffs(cOrder);
    const int K = Time.size(); 

    // init:
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(K, 3 * N);

    // tic:
    const auto tStart = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < Pos.cols(); ++i) {
        switch (method) {
            case TrajectoryOptimizer::Solver::Numeric:
                result.block(0, i*N, K, N) = DoTrajectoryGenerationNumerically(
                    tOrder,
                    cOrder,
                    Pos.col(i),
                    Vel.col(i),
                    Acc.col(i),
                    Time
                );
                break;
            case TrajectoryOptimizer::Solver::Analytic:
                result.block(0, i*N, K, N) = DoTrajectoryGenerationAnalytically(
                    tOrder,
                    cOrder,
                    Pos.col(i),
                    Vel.col(i),
                    Acc.col(i),
                    Time
                );
                break;
            default:
                break;
        }
    }

    // toc:
    const auto tEnd = std::chrono::high_resolution_clock::now();

    // measure elapsed time:
    std::chrono::duration<double, std::milli> durationMs = tEnd - tStart;

    ROS_WARN(
        "[TrajectoryOptimizer::GenerateTrajectory] time elapsed %.2f ms", 
        durationMs.count()
    );

    return result;
}

/**
  * @brief evaluate d-th order derivative of trajectory polynomial at time t
  *
  * @param[in] coeffs polynomial coefficients
  * @param[in] d derivative order
  * @param[in] t target timestamp
  *
  * @return result waypoint
  */
double TrajectoryOptimizer::EvaluatePoly(const Eigen::VectorXd &coeffs, const int d, const double t) {
    const int N = coeffs.size();

    // calculate t-exponentials:
    Eigen::VectorXd powers = Eigen::VectorXd::Ones(N - d);
    for (int n = 1; n < N - d; ++n) {
        powers(n) = t*powers(n - 1);
    }

    // get result:
    double result{0.0}; 
    for (int n = d; n < N; ++n) {
      result += GetFactorial(n, d) * coeffs(n) * powers(n - d);
    }

    return result;
}

Eigen::VectorXd TrajectoryOptimizer::DoSegmentTrapezoidalTimesAllocation(
  const Eigen::MatrixXd &Path,
  const double velLimit,
  const double accLimit
) { 
    const int K = (Path.rows() - 1);
    
    Eigen::VectorXd time = Eigen::VectorXd::Zero(K);
    
    /*
        STEP 1: use "trapezoidal velocity" heuristic for time allocation
    */
    const double constAccDisplacement = (
        (velLimit * velLimit ) / (2.0 * accLimit)
    );

    // calculate segment  displacement:
    Eigen::VectorXd segDisplacement = Eigen::VectorXd::Zero(K);
    for (int k = 1; k <= K; ++k) {
        // calculate segment displacement:
        segDisplacement(k - 1) = (Path.row(k) - Path.row(k - 1)).norm();
    }

    //
    // do time allocation:
    //
    double maxVel{0.0};
    double segTime{0.0};
    for (int k = 0; k < K; ++k) {
        // CASE 1: trapezoidal is reduced to triangle
        if (segDisplacement(k) <= 2.0 * constAccDisplacement) {
            // set max. vel:
            maxVel = std::sqrt(accLimit * segDisplacement(k));

            // set total duration:
            segTime = 2.0 * (maxVel / accLimit);
        } 
        // CASE 2: stay with trapezoidal 
        else {
            // set max. vel:
            maxVel = velLimit;

            // set total duration:
            segTime = 2.0 * (maxVel / accLimit) + (segDisplacement(k) - 2.0 * constAccDisplacement) / velLimit;
        }

        // allocate time:
        time(k) = segTime;
    }

    return time;
}

Eigen::VectorXd TrajectoryOptimizer::DoGlobalTrapezoidalTimesAllocation(
  const Eigen::MatrixXd &Path,
  const double velLimit,
  const double accLimit
) {
    const int K = (Path.rows() - 1);
    
    Eigen::VectorXd time = Eigen::VectorXd::Zero(K);
    
    /*
        STEP 1: use "trapezoidal velocity" heuristic for time allocation
    */
    const double constAccDisplacement = (
        (velLimit * velLimit ) / (2.0 * accLimit)
    );

    // calculate segment / accumulated displacement:
    double totalDisplacement{0.0};
    Eigen::VectorXd segDisplacement = Eigen::VectorXd::Zero(K);
    for (int k = 1; k <= K; ++k) {
        // calculate segment displacement:
        segDisplacement(k - 1) = (Path.row(k) - Path.row(k - 1)).norm();
        // update accumulated displacement:
        totalDisplacement += segDisplacement(k - 1);
    }

    //
    // init time allocation:
    //
    double totalTime{0.0};
    double maxVel{velLimit};
    double accDisplacement{0.0}, decDisplacement{0.0};

    double currDisplacement{0.0}, nextDisplacement{0.0};
    double currTime{0.0}, nextTime{0.0};

    // CASE 1: trapezoidal is reduced to triangle
    if (totalDisplacement <= 2.0 * constAccDisplacement) {
        // set phase transition displacements:
        accDisplacement = decDisplacement = (totalDisplacement / 2.0);

        // set max. vel:
        maxVel = std::sqrt(2.0 * accLimit * decDisplacement);

        // set total duration:
        totalTime = 2.0 * (maxVel / accLimit);
    } 
    // CASE 2: stay with trapezoidal 
    else {
        // set phase transition displacements:
        accDisplacement = constAccDisplacement;
        decDisplacement = totalDisplacement - constAccDisplacement;

        // set max. vel:
        maxVel = velLimit;

        // set total duration:
        totalTime = 2.0 * (velLimit / accLimit) + (totalDisplacement - 2.0 * constAccDisplacement) / velLimit;
    }

    //
    // do time allocation:
    //
    for (int k = 0; k < K; ++k) {
        // generate proposal:
        nextDisplacement = currDisplacement + segDisplacement(k);

        // STAGE 1: acceleration
        if (nextDisplacement <= accDisplacement) {
            double endVel = std::sqrt(2.0 * accLimit * nextDisplacement);
            nextTime = endVel / accLimit;
        } 
        // STAGE 2: constant velocity
        else if (nextDisplacement <= decDisplacement) {
            nextTime = (maxVel / accLimit) + (nextDisplacement - accDisplacement) / maxVel;
        }
        // STAGE 3: deceleration
        else {
            double endVel = std::sqrt(2.0 * accLimit * (totalDisplacement - nextDisplacement));
            nextTime = totalTime - (endVel / accLimit);
        }

        // allocate time:
        time(k) = nextTime - currTime;

        // proceed to next segment;
        currDisplacement = nextDisplacement;
        currTime = nextTime;
    }

    return time;    
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
Eigen::MatrixXd TrajectoryOptimizer::DoTrajectoryGenerationNumerically(
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
    const auto C = (
        // 1. boundary value equality constraints:
        N + 
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
    
    auto GetAFactorialKey = [&](const int n, const int d) {
        return d * N + n;
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
        int currentConstraintIdx{0};

        //
        // 3.1 boundary value equality constraints:
        //

        // init:
        Eigen::MatrixXd boundaryValues = Eigen::MatrixXd::Zero(2, cOrder + 1);
        boundaryValues(0, 0) = Pos(0);
        boundaryValues(0, 1) = Vel(0);
        boundaryValues(0, 2) = Acc(0);
        boundaryValues(1, 0) = Pos(K);
        boundaryValues(1, 1) = Vel(1);
        boundaryValues(1, 2) = Acc(1);

        // populate constraints:
        for (int c = 0; c <= cOrder; ++c) {
            // start waypoint:
            ATriplets.emplace_back(
                currentConstraintIdx, c, AFactorial[GetAFactorialKey(c, c)]/ATimePower[c](0)
            );
            b(currentConstraintIdx) = boundaryValues(0, c);
            
            ++currentConstraintIdx;

            // end waypoint:
            for (int n = c; n < N; ++n) {
                ATriplets.emplace_back(
                    currentConstraintIdx, (K - 1)*N + n, AFactorial[GetAFactorialKey(n, c)]/ATimePower[c](K - 1)
                );
            }
            b(currentConstraintIdx) = boundaryValues(1, c);

            ++currentConstraintIdx;
        }

        //
        // 3.2 intermediate waypoint passing equality constraints:
        //
        for (int k = 1; k < K; ++k) {
            ATriplets.emplace_back(
                currentConstraintIdx, k*N, 1.0
            );
            b(currentConstraintIdx) = Pos(k);

            ++currentConstraintIdx;
        }

        //
        // 3.3 intermediate waypoint continuity constaints:
        //
        for (int c = 0; c <= cOrder; ++c) {
            for (int k = 1; k < K; ++k) {
                // the end of previous trajectory segment:
                for (int n = c; n < N; ++n) {
                    ATriplets.emplace_back(
                        currentConstraintIdx, 
                        (k - 1) * N + n,
                        AFactorial[GetAFactorialKey(n, c)]/ATimePower[c](k - 1)
                    );
                }

                // should equal to the start of current trajectory segment:
                ATriplets.emplace_back(
                    currentConstraintIdx,
                    k * N + c,
                    -AFactorial[GetAFactorialKey(c, c)]/ATimePower[c](k) 
                );

                // set next constraint:
                ++currentConstraintIdx;
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
            Eigen::VectorXd timeScalings = Eigen::VectorXd::Ones(N);
            for (int k = 0; k < K; ++k) {
                for (int n = 1; n < N; ++n) {
                    timeScalings(n) = timeScalings(n - 1) / Time(k);
                }
                result.row(k) = optimalCoeffs.segment(k * N, N).cwiseProduct(timeScalings);;
            }

            ROS_WARN("[Minimum Snap, Numeric]: Optimal objective is %.2f.", optimalObject);
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
Eigen::MatrixXd TrajectoryOptimizer::DoTrajectoryGenerationAnalytically(
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
    // dim of flattened output, direct:
    const auto D = K * N;
    // dim of flattened output, latent:
    const auto L = (K + 1) * (cOrder + 1);
    const auto LFixed = (
        // 1. boundary value equality constraints:
        N + 
        // 2. intermediate waypoint passing equality constraints:
        (K - 1)
    );
    const auto LVariable = L - LFixed;

    //
    // init output:
    //
    //     the trajectory segment k is defined by result(k, 0) + t*(result(k, 1) + ...)
    // 
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(K, N);

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
    // define derivative matrix M:
    //
    // 1. init:
    Eigen::SparseMatrix<double> M(D, D);
    std::vector<Eigen::Triplet<double>> MTriplets;

    // 2. cache results for constraint matrix construction:
    std::map<int, Eigen::VectorXd> MTimePower;
    std::map<int, double> MFactorial;

    // 2.a init:
    MTimePower.insert(
        {0, Eigen::VectorXd::Ones(K)}
    );
    
    auto GetMFactorialKey = [&](const int n, const int d) {
        return d * N + n;
    };
    for (int n = 0; n < N; ++n) {
        MFactorial.insert(
            {GetMFactorialKey(n, 0), 1.0}
        );
    }

    for (int c = 1; c <= cOrder; ++c) {
        // 2.b time power pre-computing:
        MTimePower.insert(
            {c, MTimePower[c - 1].cwiseProduct(Time)}
        );

        // 2.c factorial pre-computing:
        for (int n = c; n < N; ++n) {
            MFactorial.insert(
                {GetMFactorialKey(n, c), (n - c + 1)*MFactorial[GetMFactorialKey(n, c - 1)]}
            );
        }
    };

    // 3. populate MTriplets:
    {
        int currentStateIndex{0};

        for (int k = 0; k < K; ++k) {
            for (int c = 0; c <= cOrder; ++c) {
                // the start state defined by current traj. seg.:
                MTriplets.emplace_back(
                    currentStateIndex++,
                    k * N + c,
                    MFactorial[GetMFactorialKey(c, c)]/MTimePower[c](k) 
                );
            }

            for (int c = 0; c <= cOrder; ++c) {
                // the end state defined by current traj. seg.:
                for (int n = c; n < N; ++n) {
                    MTriplets.emplace_back(
                        currentStateIndex, 
                        k * N + n,
                        MFactorial[GetMFactorialKey(n, c)]/MTimePower[c](k)
                    );
                }
                ++currentStateIndex;
            }
        }
    }

    // 4. populate M:
    M.setFromTriplets(std::begin(MTriplets),std::end(MTriplets));

    // 5. get inverse M:
    // 5.1 decompose:
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(M);
    if(solver.info() != Eigen::Success) {
        // decomposition failed
        ROS_WARN("[Minimum Snap, Analytic]: Failed to decompose derivative matrix M.");
    }
    // 5.2 inverse:
    Eigen::SparseMatrix<double> I(D,D);
    I.setIdentity();
    auto MInv = solver.solve(I);
    if(solver.info() != Eigen::Success) {
        // matrix inverse failed
        ROS_WARN("[Minimum Snap, Analytic]: Failed to compute the inverse derivative matrix M.");
    }

    //
    // define selection matrix C:
    //
    // 1. init:
    Eigen::SparseMatrix<double> C(D, L);
    std::vector<Eigen::Triplet<double>> CTriplets;

    // 2. populate CTriplets:
    {
        // 
        // latent variable definition L is defined as LFixed plus LVariable
        //
        int currentStateIndex{0};

        // 2.1 map fixed latent variables, boundary value constraints:
        for (int c = 0; c <= cOrder; ++c) {
            CTriplets.emplace_back(
                c,
                currentStateIndex,
                1.0
            );
            ++currentStateIndex;

            CTriplets.emplace_back(
                (K - 1) * N + (cOrder + 1 + c),
                currentStateIndex,
                1.0
            );
            ++currentStateIndex;
        }

        // map the rest:
        for (int c = 0; c <= cOrder; ++c) {
            // 2.2 intermediate waypoint passing constraints will be mapped when c = 0:
            // 2.3 actual decision latent variables will be mapped when c > 0:
            for (int k = 1; k < K; ++k) {
                CTriplets.emplace_back(
                    (k - 1) * N + (cOrder + 1 + c),
                    currentStateIndex,
                    1.0
                );

                CTriplets.emplace_back(
                    k * N + c,
                    currentStateIndex,
                    1.0
                );
                
                ++currentStateIndex;
            }
        }
    }

    // 4. populate C:
    C.setFromTriplets(std::begin(CTriplets),std::end(CTriplets));

    //
    // build problem:
    //
    // 1. define fixed variables:
    Eigen::VectorXd outputLatent = Eigen::VectorXd::Zero(L);
    // 1.a boundary value equality constraints:
    outputLatent(0) = Pos(0);
    outputLatent(1) = Pos(1);
    outputLatent(2) = Vel(0);
    outputLatent(3) = Vel(1);
    outputLatent(4) = Acc(0);
    outputLatent(5) = Acc(1);
    // 1.b intermediate waypoint passing equality constraints:
    for (int k = 1; k < K; ++k) {
        outputLatent(N + k - 1) = Pos(k);
    }

    // 2. define A & b for least squared:
    Eigen::MatrixXd PDense(C.transpose() * MInv.transpose() * P * MInv * C);

    const Eigen::MatrixXd& A = PDense.block(LFixed, LFixed, LVariable, LVariable);
    const Eigen::VectorXd b = -PDense.block(LFixed,      0, LVariable,    LFixed)*outputLatent.segment(0, LFixed);

    // 3. get latent output:
    outputLatent.segment(LFixed, LVariable) = A.colPivHouseholderQr().solve(b);

    // 4. get direct output:
    const Eigen::VectorXd outputDirect = MInv * C * outputLatent;

    // 5. get optimal objective:
    const auto optimalObject = (0.50 * outputDirect.transpose() * P * outputDirect);
    ROS_WARN("[Minimum Snap, Analytic]: Optimal objective is %.2f.", optimalObject(0, 0));

    //
    // format output:
    //
    Eigen::VectorXd timeScalings = Eigen::VectorXd::Ones(N);
    for (int k = 0; k < K; ++k) {
        for (int n = 1; n < N; ++n) {
            timeScalings(n) = timeScalings(n - 1) / Time(k);
        }
        result.row(k) = outputDirect.segment(k * N, N).cwiseProduct(timeScalings);;
    }

    // done:
    return result;
}
