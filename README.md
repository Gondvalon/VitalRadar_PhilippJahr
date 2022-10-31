# VitalRadar_PhilippJahr

All folders are rospackages

gazebo_ros_vital_radar has the vital radar plugin 
	which detects vital signs of other objects.
	
ec_scout_model has the standard ec_scout model
	 with the gazebo sensor added to the 
	 ec_scout_sensor_head.urdf robot description
	
vital_radar_scenarios has the scenarios that were used for
	the evaluation. The model which resembles a human is
	included as well.
	
msg_subscriber has a simple subscriber for the vital radar
	which is suitable for the scenarios. Depending on the
	choosen scenario the subscriber has to be configurated.
