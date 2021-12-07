1. The entrance function is in pso/pso_test_obstacle.m

2. Your task is to complete the pso_select.m function inside the "To be finished by the student" part.

3. The input to the pso_select function are:
   theta (1 x 1): current heading of the vehicle
   omega (1 x 1): current angular speed of the vehicle's heading
   v_ini (1 x 1): current forward speed of the vehicle
   p0 (2 x 1): current position of the vehicle [x;y]
   last_theta (1 x 1): previous heading target
   last_v (1 x 1): previous speed target

4. The output to the pso_select function:
global_best (1 x 3): is a vector consists of [theta_target, velocity_target, best_cost] 

5. The rest of the code will automatically generate the trajectory when you pass it the global_best.
