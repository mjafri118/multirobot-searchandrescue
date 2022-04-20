
# Command lines to run for each robot
# ON ROBOT:

# Bringing up the searcher robot (locobot3)
- roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100 use_nav:=true use_lidar:=true rtabmap_args:=-d robot_name:=locobot3

# ON PC:

- python waypoint.py --robot_name locobot3

- python searcherFSM.py --robot_index 0

# Bringing up RVIZ
- roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot3/map robot_name:=locobot3