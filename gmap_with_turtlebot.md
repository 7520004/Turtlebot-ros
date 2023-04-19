
################################################################
★	Turtlebotによるgmappingの操作コマンド
################################################################

[コマンド]
	#joy
	$ roslaunch turtlebot_le2i gmap_with_turtlebot.launch
	#key
	$ roslaunch turtlebot_le2i gmap_with_turtlebot.launch mode:=key
	
[内容]
#Turtlebot起動
	$ roslaunch turtlebot_bringup minimal.launch
#rplidar起動
	$ roslaunch rplidar_ros rplidar_s1.launch
#gmapping起動(odom指定)
	$ rosrun gmapping slam_gmapping _odom_frame:=odom
#rviz_with_robot起動
	$ roslaunch turtlebot_le2i view_robot_rplidar.launch
#teleop起動
	#key
	$ roslaunch turtlebot_teleop keyboard_teleop.launch
	#joy
	$ roslaunch turtlebot_teleop joy_teleop.launch


[参考文献]
AGIRobots - https://agirobots.com/rplidar-intro3/
