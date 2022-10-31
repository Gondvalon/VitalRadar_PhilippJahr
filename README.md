# VitalRadar_PhilippJahr

All folders are rospackages

gazebo_ros_vital_radar has the vital radar plugin 
	which detects vital signs of other objects.
	
vital_radar_scenarios has the scenarios that were used for
	the evaluation. The model which resembles a human is
	included as well.
	
msg_subscriber has a simple subscriber for the vital radar
	which is suitable for the scenarios. Depending on the
	choosen scenario the subscriber has to be configurated.
	
gazebo_vital_radar_sensor holds the description which has to be
	included in the robot the radar shall be used with. A joint
	to the desired location has to be added then as well.
