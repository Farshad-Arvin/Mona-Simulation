#BY: SARIKA RAMROOP
#DATE LAST MODIFIED: 14/08/2017

#include file for Mona scaled to actual size

#define IR sensor
define mona_ir sensor
(
  #transducer size
  size [0.004 0.007 0.0025]
  
  #range thresholds [min max]
  range [0 0.045] # this needs to be changed

  #angular field of view in degrees
  fov 80  # this is a temp value. subject to change
  samples 10

  # define the color that ranges are drawn in the gui
  color_rgba [ 1 0 1 0.5 ]
)

# Sensor array
define mona_irs ranger
(
  # one line for each sonar [xpos ypos zpos heading]
  mona_ir( pose [0.029 0.0203125 0 40] ) 	
  mona_ir( pose [0.0325 0 0 0] )		
  mona_ir( pose [0.029 -0.0203125 0 -40] )
)

#defining the blobfinder.Used to measure "intensity"
define colour_check blobfinder
(
	#number of colours that can be detected
	colors_count 11
	
	#which colours to look for
	colors ["gray100" "gray90" "gray80" "gray70" "gray60" "gray50" "gray40" "gray30" "gray20" "gray10" "black"]	

	#camera parameters
	image [30 15]		#resolution in pixels
	range 0.02			#range in meters
	fov 180				#angular range
	color_rgba [ 1 0 1 0.5 ]
)

define  intensity ranger
(
	colour_check( pose [0.0325 0.0 0 0] )		# below centre IR sensor
	size [ 0.0 0.0 0.0 ]
)

define detect fiducial 
(
	range_min 0.0			#min range 
	range_max 0.045			#max range 
	range_max_id 0.04		#max range at which it can detect an ID 
	fov 180
	
	ignore_zloc 1 			#sees objects at a lower level
	
	size [0.004 0.007 0.0025]
)

# block for paper that was used at the back of 
# the robot. 
define paper model
(
	size [ 0.010 0.041 0.028]
	block(	
		points 11
		point[0] [0.021 0.041]
		point[1] [0.028 0.039]
		point[2] [0.031 0.034]
		point[3] [0.034 0.029]
		point[4] [0.036 0.023]
		point[5] [0.036 0.0205]
		point[6] [0.036 0.0163]
		point[7] [0.034 0.0098]
		point[8] [0.031 0.005]
		point[9] [0.026 0.0016]
		point[10] [0.021 0.0 ]
		z [0 0.028]
	)
)

# Block for the motor and battery covering
define motor model
(
	size [ 0.021 0.041 0.028]
	block(
		points 4
		point[0] [0 0]
		point[1] [0 0.021]
		point[2] [ 0.041 0.021]
		point[3] [ 0.041 0 ]
		z [0 0.028]
	)
)

#wheels
define wheel model
(
	size [ 0.035 0.005 0.035 ]

	#color "blue"	

	block(
		points 4
    		point[0] [0.0 0.0]
    		point[1] [0.035 0.0]
    		point[2] [0.035 0.005]
    		point[3] [0.0 0.005]
      		z [0 0.035]
	)
)

# IR sensor casing 
# Physical representation of the sensors. 
define case model
(
	size [0.0025 0.007 0.004 ]
	block
	(
		points 4
		point[0] [0 0]
		point[1] [0 0.007]
		point[2] [0.0025 0.007]
		point[3] [0.0025 0]
		z [0 0.004]
	)
	fiducial_return 1 				#returns ID 1 to a fiducial module
)

#define the robot's body
define mona_blob position
(
  #actual size in metres
  size [0.065 0.065 0.01] # constrained to 100cm x 100cm

  #centre of rotation offset
  #origin [0 0.5 0 0]

  # shape of the robot
  block
  (
    points 22 #number of points; circle estimate
    point[0] [0 0]
    point[1] [0.000813 0.0065]
    point[2] [0.00244 0.013]
    point[3] [0.00569 0.0195]
    point[4] [0.012188 0.026]
    point[5] [0.01625 0.02925]
    point[6] [0.04794 0.02925]
    point[7] [0.052 0.026]
    point[8] [0.0585 0.0195]
    point[9] [0.06175 0.013]
    point[10] [0.064188 0.0065]
    point[11] [0.065 0]
    point[12] [0.064188 -0.0065]
    point[13] [0.06175 -0.013]
    point[14] [0.0585 -0.0195]
    point[15] [0.052 -0.026]
    point[16] [0.04794 -0.02925]
    point[17] [0.01625 -0.02925]
    point[18] [0.012188 -0.026]
    point[19] [0.00569 -0.0195]
    point[20] [0.00244 -0.013]
    point[21] [0.000813 -0.0065]
    z [0 0.01]
  )

  # Differential drive
  drive "diff"

  obstacle_return 1				# detected as an obstacle
  ranger_return 0.5				# reflect ir signals
  blob_return 0               	# Seen by blobfinders 

  # sensors
  mona_irs()						# IR
  intensity()						# light
  detect( pose [ 0.0325 0 0.11 0 ])	# robot ID
 
  #other blocks
  motor( pose [-0.011 0 0.0 0] color "orange" )
  paper( pose [-0.027 0 0.0 -180] color "orange" )
  wheel( pose [-0.01 -0.035 -0.01 0] )
  wheel( pose [-0.01 0.035 -0.01 0] )
  case( pose [0.0275 0.0203125 0 40] ) 
  case( pose [0.0325 0 0 0] )		
  case( pose [0.0275 -0.0203125 0 -40] )
)