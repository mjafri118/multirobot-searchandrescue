
# Command lines to run for each robot
# Starting WSR AND SLAM FOR TX10, LOCOBOT3

# Step 1 PC:

- roscore

# Step 2 PC:

- roslaunch wsr_toolbox_cpp wsr_all.launch config_fn:=${HOME}/Harvard_CS286/cs286_mini_hack_2/cs286_config_live.json ws_name:=cs286_hack_ws d_type:=odom

# Step 3 PC:

- python ~/Harvard_CS286/cs286_mini_hack_2/scripts/online_data_test2.py --robot_username all-locobot3 --robot_ip 192.168.1.13 --tx_username all-up-tx10 --tx_ip 192.168.1.30 --packet_len 65 --ts 5

# Step 4 ROBOT:

- roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100 use_nav:=true use_lidar:=true rtabmap_args:=-d robot_name:=locobot3

# Step 5 PC:

- roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot3/map robot_name:=locobot3

# Step 6 ROBOT:

- python ~/Harvard_CS286/cs286_mini_hack_2/scripts/collect_robot_displacement.py --position_topic /locobot3/mobile_base/odom --velocity_topic /locobot3/mobile_base/commands/velocity --motion curved

# Step 7 PC:

- python waypoint.py --robot_name locobot3

# Step 8 PC: (OPTIONAL)

OPTIONALLY: TRYING TO RUN ON PC
- roslaunch wsr_toolbox_cpp wsr_check_csi.launch config_fn:=${HOME}/Harvard_CS286/cs286_mini_hack_2/cs286_config_live.json ws_name:=cs286_hack_ws

# Step 9: PC
- - python searcherFSM.py --robot_index 0