#planner stats
float64 initial_epsilon
float64 final_epsilon
bool plan_to_first_solution
float64 allocated_time
float64 actual_time
float64 time_to_first_solution
float64 solution_cost
float64 path_size
int64 final_number_of_expands
int64 number_of_expands_initial_solution

#problem stats
geometry_msgs/PoseStamped start
float64 start_cart_angle
geometry_msgs/PoseStamped goal
float64 goal_cart_angle

#solution
cart_pushing_msgs/RobotCartPath solution