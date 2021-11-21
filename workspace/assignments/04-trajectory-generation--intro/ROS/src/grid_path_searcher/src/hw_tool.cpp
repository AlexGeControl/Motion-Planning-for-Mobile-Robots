#include <hw_tool.h>

#include <limits>

#include <unsupported/Eigen/Polynomials>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    static const double epsilon = 0.001;

    double optimal_cost = std::numeric_limits<double>::max(); 
    
    /*
        STEP 2: get OBVP cost
    */

    // init:
    const auto &pos_x_str = _start_position.x();
    const auto &pos_y_str = _start_position.y();
    const auto &pos_z_str = _start_position.z();

    const auto pos_x_str_squared = pos_x_str * pos_x_str;
    const auto pos_y_str_squared = pos_y_str * pos_y_str;
    const auto pos_z_str_squared = pos_z_str * pos_z_str;

    const auto &vel_x_str = _start_velocity.x();
    const auto &vel_y_str = _start_velocity.y();
    const auto &vel_z_str = _start_velocity.z();

    const auto vel_x_str_squared = vel_x_str * vel_x_str;
    const auto vel_y_str_squared = vel_y_str * vel_y_str;
    const auto vel_z_str_squared = vel_z_str * vel_z_str;

    const auto &pos_x_end = _target_position.x();
    const auto &pos_y_end = _target_position.y();
    const auto &pos_z_end = _target_position.z();

    const auto pos_x_end_squared = pos_x_end * pos_x_end;
    const auto pos_y_end_squared = pos_y_end * pos_y_end;
    const auto pos_z_end_squared = pos_z_end * pos_z_end;

    const auto &vel_x_end = _start_velocity.x();
    const auto &vel_y_end = _start_velocity.y();
    const auto &vel_z_end = _start_velocity.z();

    const auto vel_x_end_squared = vel_x_end * vel_x_end;
    const auto vel_y_end_squared = vel_y_end * vel_y_end;
    const auto vel_z_end_squared = vel_z_end * vel_z_end;

    // define cost function:
    auto EvaluateCost = [&](const double T) {
        auto result = (
            T * (
                T*(
                    T + 
                    (4*vel_x_end_squared + 4*vel_x_end*vel_x_str + 4*vel_x_str_squared + 4*vel_y_end_squared + 4*vel_y_end*vel_y_str + 4*vel_y_str_squared + 4*vel_z_end_squared + 4*vel_z_end*vel_z_str + 4*vel_z_str_squared)
                ) + 
                (-12*pos_x_end*vel_x_end - 12*pos_x_end*vel_x_str + 12*pos_x_str*vel_x_end + 12*pos_x_str*vel_x_str - 12*pos_y_end*vel_y_end - 12*pos_y_end*vel_y_str + 12*pos_y_str*vel_y_end + 12*pos_y_str*vel_y_str - 12*pos_z_end*vel_z_end - 12*pos_z_end*vel_z_str + 12*pos_z_str*vel_z_end + 12*pos_z_str*vel_z_str)
            ) +  
            (12*pos_x_end_squared - 24*pos_x_end*pos_x_str + 12*pos_x_str_squared + 12*pos_y_end_squared - 24*pos_y_end*pos_y_str + 12*pos_y_str_squared + 12*pos_z_end_squared - 24*pos_z_end*pos_z_str + 12*pos_z_str_squared)
        );

        return result / std::pow(T, 3);
    };
    


    // identify where the derivative of cost function equals to 0:
    Eigen::VectorXd coeffs(7);

    coeffs << 0.0,
           // 1st:
              0.0,
           // 2nd:
              -36*pos_x_end_squared + 72*pos_x_end*pos_x_str - 36*pos_x_str_squared - 36*pos_y_end_squared + 72*pos_y_end*pos_y_str - 36*pos_y_str_squared - 36*pos_z_end_squared + 72*pos_z_end*pos_z_str - 36*pos_z_str_squared,
           // 3rd:
              24*pos_x_end*vel_x_end + 24*pos_x_end*vel_x_str - 24*pos_x_str*vel_x_end - 24*pos_x_str*vel_x_str + 24*pos_y_end*vel_y_end + 24*pos_y_end*vel_y_str - 24*pos_y_str*vel_y_end - 24*pos_y_str*vel_y_str + 24*pos_z_end*vel_z_end + 24*pos_z_end*vel_z_str - 24*pos_z_str*vel_z_end - 24*pos_z_str*vel_z_str,
           // 4th:
              -4*vel_x_end_squared - 4*vel_x_end*vel_x_str - 4*vel_x_str_squared - 4*vel_y_end_squared - 4*vel_y_end*vel_y_str - 4*vel_y_str_squared - 4*vel_z_end_squared - 4*vel_z_end*vel_z_str - 4*vel_z_str_squared,
           // 5th:
              0.0,
           // 6th:
              1.0;

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);

    const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &r = solver.roots();
    
    ROS_WARN("[OBVP Solver]: activated...");
    for (size_t i = 0; i < 6; ++i) {
        if (
            // positive real root only:
            (r(i).real() > epsilon) &&
            (std::abs(r(i).imag()) < epsilon)
        ) {
            // log root:
            ROS_WARN(
                "\tcandidate root, %dth: %.2f + i%.2f", 
                static_cast<int>(i + 1), 
                r(i).real(), 
                r(i).imag()
            );
            
            double curr_cost = EvaluateCost(r(i).real());

            // log cost:
            ROS_WARN(
                "\tstate transition cost: %.2f", curr_cost
            );

            if (curr_cost < optimal_cost) {
                optimal_cost = curr_cost;
            }
        }
        
    }
    ROS_WARN("[OBVP Solver]: done! optimal cost is: %.2f\n", optimal_cost);

    return optimal_cost;
}
