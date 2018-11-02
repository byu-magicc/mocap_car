# New waypoint, input to path manager

# @warning w and Va_d always have to be valid; the chi_d is optional.
float32[2] w		# Waypoint in local NE (m)
bool set_current	# Sets this waypoint to be executed now! Starts a new list
bool clear_wp_list  # Removes all waypoints and returns to origin.  The rest of
                    # this message will be ignored
