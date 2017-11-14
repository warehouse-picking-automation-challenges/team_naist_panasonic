# all poses are in the camera frame
string item_id
float32 confidence
float32 center_x  	# in image frame
float32 center_y  	# in image frame
float32 width 		# in image frame
float32 height 		# in image frame

geometry_msgs/PoseStamped poseCenter   	# center of the bounding box in camera frame
geometry_msgs/PoseStamped poseGrasp    	# center of the grasp line in camera frame
geometry_msgs/PoseStamped graspLinePt1 	# first grasp line extremity using depth of the grasp point plane
geometry_msgs/PoseStamped graspLinePt2 	# second grasp line extremity using depth of the grasp point plane
