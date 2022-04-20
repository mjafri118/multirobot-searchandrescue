
# Bringing up the searcher robot (locobot4)

- roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100 use_nav:=true use_lidar:=true rtabmap_args:=-d robot_name:=locobot4

- python waypoint.py --robot_name locobot4

# Starting RVIZ for (locobot4) - remember to change all topics

- roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot4/map robot_name:=locobot4