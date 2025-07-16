# manymove parameters
float64 max_cartesian_speed             # max cartesian speed for the move, referred to TCP
int32 plan_number_target                # number of valid plans to find before declaring success
int32 plan_number_limit                 # max planning tries before fail
string smoothing_type                   # "time-optimal" / "ruckig"
string tcp_frame                        # End-effector (TCP) frame for this request
float64 linear_precision                # Linear precision for end-point trajectory position check
float64 rotational_precision            # Rotational precision for end-point trajectory position check
float64 deceleration_time               # Time to decelerate the robot to arrive at a full stop
float64 min_stop_time                   # Minimum time to perform a controlled stop, if less time remains the traj will just end
# moveit planner parameters
float64 velocity_scaling_factor         # 0.0 to 1.0
float64 acceleration_scaling_factor     # 0.0 to 1.0
string planning_pipeline                # e.g. "ompl", "chomp", "pilz_industrial_motion_planner", ...
string planner_id                       # e.g. "RRTConnect", "PTP", "LIN", ...
float64 planning_time                   # overall time budget in seconds
int32  planning_attempts                # e.g. 1, 5, etc.
# CartesianInterpolator parameters
float64 step_size
float64 cartesian_precision_translational 0.001
float64 cartesian_precision_rotational 0.01
float64 cartesian_precision_max_resolution 0.001