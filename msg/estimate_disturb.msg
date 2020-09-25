# Estimated external disturbance(force or torque)

uint64 timestamp		# time since system start (microseconds)

# Estimated disturbance in body frame
float32[3] esti_dist

# Intermediate variables
float32[3] ft_hat
float32[3] ft_motor
float32[3] ft_motor_filter

# TOPICS estimate_disturb estimate_force estimate_torque

