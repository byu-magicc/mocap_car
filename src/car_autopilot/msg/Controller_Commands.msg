# Controller commands output from the path follower, input to autopilot controller

# @warning u_c and psi_c have always to be valid, the aux array is optional
float32 u_c		# Commanded airspeed (m/s)
float32 psi_c		# Commanded heading (rad)
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid
