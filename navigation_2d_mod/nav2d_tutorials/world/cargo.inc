define cargofeet model
(
	color "green"				# Default color.
	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0
	
	gripper_return 0
	fiducial_return 0
	laser_return 1			# reflects laser beams
	ranger_return 200		# reflects intensity

	# Actual size
	size [0.04 0.04 1]

	# Body shape:
	block
	( 
		points 4
		point[0] [-0.02 0.02]
		point[1] [-0.02 -0.02]
		point[2] [0.02 -0.02]
		point[3] [0.02 0.02]
		z [0 1]
	)
)
