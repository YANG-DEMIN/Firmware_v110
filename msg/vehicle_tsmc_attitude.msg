# Main intermediate variables of TSMC-based controller(Attitude Loop)
uint64 timestamp		# time since system start (microseconds)

# Equivalent control part
float32 eq_part_roll
float32 eq_part_pitch
float32 eq_part_yaw

# Discontinuous control part
float32 disc_part_roll
float32 disc_part_pitch
float32 disc_part_yaw

# sign of S
float32 sign_s_roll
float32 sign_s_pitch
float32 sign_s_yaw

# angular rates error
float32 rates_err_roll
float32 rates_err_pitch
float32 rates_err_yaw
