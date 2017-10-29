##################################
# Description: File containing the
# different "light" gradients.
#
# By: Sarika Ramroop
##################################

#---------------------------------
# Bright light source for large
# arena
#---------------------------------

define shade1 model
(
	bitmap "../bitmaps/circle.png"	#Colors this image 
	size [0.1 0.1 0]
	color "white"	
)

define shade2 model
(
	bitmap "../bitmaps/circle.png"
	size [0.2 0.2 0]
	color "white"		
)

define shade3 model
(
	bitmap "../bitmaps/circle.png"
	size [0.3 0.3 0]
	color "gray80"		
)

define shade4 model
(
	bitmap "../bitmaps/circle.png"
	size [0.4 0.4 0]
	color "gray70"
)

define shade5 model
(
	bitmap "../bitmaps/circle.png"
	size [0.5 0.5 0]
	color "gray60"
)

define shade6 model
(
	bitmap "../bitmaps/circle.png"
	size [0.6 0.6 0]
	color "gray50"
)

define shade7 model
(
	bitmap "../bitmaps/circle.png"
	size [0.7 0.7 0]	
	color "gray40"
)

define shade8 model
(
	bitmap "../bitmaps/circle.png"
	size [0.8 0.8 0]
	color "gray30"
)

define shade9 model
(
	bitmap "../bitmaps/circle.png"
	size [0.9 0.9 0]
	color "gray20"
)

#define shade10 model
#(
#	bitmap "../bitmaps/circle.png"
#	size [1 1 0]
#	color "gray10" 
#)
#
#define shade11 model
#(
#	bitmap "../bitmaps/circle.png"
#	size [1.1 1.1 0]
#	color "black"
#)


define gradient model
(
	# most maps will need a bounding box
	boundary 0
	
	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0
	gripper_return 0
	fiducial_return 0
	# insensible to collision and range sensors
	obstacle_return 0
	ranger_return 1

	shade1(obstacle_return 0)
	shade2(obstacle_return 0)
	shade3(obstacle_return 0)
	shade4(obstacle_return 0)
	shade5(obstacle_return 0)
	shade6(obstacle_return 0)
	shade7(obstacle_return 0)
	shade8(obstacle_return 0)
	shade9(obstacle_return 0)
#	shade10(obstacle_return 0)
#	shade11(obstacle_return 0)
)

#---------------------------------
# dim gradient - large arena
#---------------------------------
define shade21 model
(
	bitmap "../bitmaps/circle.png"
	size [0.1 0.1 0]
	color "white"	
)

define shade22 model
(
	bitmap "../bitmaps/circle.png"
	size [0.2 0.2 0]
	color "gray80"		
)

define shade23 model
(
	bitmap "../bitmaps/circle.png"
	size [0.3 0.3 0]
	color "gray60"		
)

define shade24 model
(
	bitmap "../bitmaps/circle.png"
	size [0.4 0.4 0]
	color "gray40"
)

define shade25 model
(
	bitmap "../bitmaps/circle.png"
	size [0.5 0.5 0]
	color "gray20"
)

define shade26 model
(
	bitmap "../bitmaps/circle.png"
	size [0.6 0.6 0]
	color "gray10"
)


define gradient_dim model
(
	# most maps will need a bounding box
	boundary 0
	
	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0
	gripper_return 0
	fiducial_return 0
	# insensible to collision and range sensors
	obstacle_return 0
	ranger_return 1

	shade21(obstacle_return 0)
	shade22(obstacle_return 0)
	shade23(obstacle_return 0)
	shade24(obstacle_return 0)
	shade25(obstacle_return 0)
	shade26(obstacle_return 0)
)


#---------------------------------
#single gradient used for Exp1
#Outer radius = 32cm (Real size)
#---------------------------------
define shade41 model
(
	bitmap "../bitmaps/circle.png"
	size [0.08 0.08 0]
	color "white"	
)

define shade42 model
(
	bitmap "../bitmaps/circle.png"
	size [0.12 0.12 0]
	color "white"		
)

define shade43 model
(
	bitmap "../bitmaps/circle.png"
	size [0.16 0.16 0]
	color "white"		
)

define shade44 model
(
	bitmap "../bitmaps/circle.png"
	size [0.20 0.20 0]
	color "gray80"
)

define shade45 model
(
	bitmap "../bitmaps/circle.png"
	size [0.24 0.24 0]
	color "gray60"
)

define shade46 model
(
	bitmap "../bitmaps/circle.png"
	size [0.28 0.28 0]
	color "gray40"
)

define shade47 model
(
	bitmap "../bitmaps/circle.png"
	size [0.320 0.320 0]
	color "gray20"
)

define gradient_single model
(
	# most maps will need a bounding box
	boundary 0
	
	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0
	gripper_return 0
	fiducial_return 0
	# insensible to collision and range sensors
	obstacle_return 0
	ranger_return 1

	shade41(obstacle_return 0)
	shade42(obstacle_return 0)
	shade43(obstacle_return 0)
	shade44(obstacle_return 0)
	shade45(obstacle_return 0)
	shade46(obstacle_return 0)
	shade47(obstacle_return 0)
)

#---------------------------------
#dim gradient - Exp2 (real size)
#---------------------------------
define shade51 model
(
	bitmap "../bitmaps/circle.png"
	size [0.04 0.04 0]
	color "white"	
)

define shade52 model
(
	bitmap "../bitmaps/circle.png"
	size [0.08 0.08 0]
	color "white"		
)

define shade53 model
(
	bitmap "../bitmaps/circle.png"
	size [0.12 0.12 0]
	color "gray80"		
)

define shade54 model
(
	bitmap "../bitmaps/circle.png"
	size [0.16 0.16 0]
	color "gray60"
)

define shade55 model
(
	bitmap "../bitmaps/circle.png"
	size [0.20 0.20 0]
	color "gray40"
)

define gradient_double model
(
	# most maps will need a bounding box
	boundary 0
	
	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0
	gripper_return 0
	fiducial_return 0
	# insensible to collision and range sensors
	obstacle_return 0
	ranger_return 1

	shade51(obstacle_return 0)
	shade52(obstacle_return 0)
	shade53(obstacle_return 0)
	shade54(obstacle_return 0)
	shade55(obstacle_return 0)
)
