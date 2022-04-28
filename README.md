# STEPS TO RUN ONCE

Connect all robots and tx nodes to the same wifi network

# On ROBOT:
enable passwordless ssh

# On TX:
enable passwordless ssh

# On PC:
nano ~/.bashrc
- change ROS_MASTER_URI to PC ip addr
- change ROS_HOSTNAME to PC ip addr
source ~/.bashrc

# On ROBOT:
nano ~/.bashrc
- change ROS_MASTER_URI to PC ip addr
- change ROS_HOSTNAME to ROBOT ip addr
source ~/.bashrc

# On ROBOT for LOCOBOT3:
cd ~/
sudo ./WSR-WifiDriver/setup.sh 100 104 HT20

# On ROBOT for LOCOBOT4:
cd ~/
sudo ./WSR-WifiDriver/setup.sh 100 108 HT20

# On ROBOT for LOCOBOT5:
cd ~/
sudo ./WSR-WifiDriver/setup.sh 100 104 HT20

# On TX9:
cd ~
sudo ./WSR-WifiDriver/setup.sh 63 104 HT20

# On TX3: (Use correct tx packet length)
cd ~
sudo ./WSR-WifiDriver/setup.sh 61 108 HT20

# On TX1: (Use correct tx packet length)
cd ~
sudo ./WSR-WifiDriver/setup.sh 57 104 HT20


# TERMINALS TO RUN EVERY TIME

# On PC:
roscore

# On TX: (Use correct tx packet length)
cd ~
sudo ./WSR-WifiDriver/setup.sh 65 104 HT20

# On PC:
roslaunch wsr_toolbox_cpp wsr_pub.launch config_fn_1:=${HOME}/Harvard_CS286/cs286_mini_hack_2/cs286_config_live_locobot3.json config_fn_2:=${HOME}/Harvard_CS286/cs286_mini_hack_2/cs286_config_live_locobot4.json config_fn_3:=${HOME}/Harvard_CS286/cs286_mini_hack_2/cs286_config_live_locobot5.json ws_name:=cs286_hack_ws d_type:=odom

# On PC: for LOCOBOT3
python ~/Harvard_CS286/cs286_mini_hack_2/scripts/online_data_test2.py --robot_username all-locobot3 --robot_ip 192.168.1.13 --tx_username all-up-tx9 --tx_ip 192.168.1.29 --packet_len 63 --robot locobot3 --ts 5

# On PC: for LOCOBOT4
python ~/Harvard_CS286/cs286_mini_hack_2/scripts/online_data_test2.py --robot_username all-locobot4 --robot_ip 192.168.1.14 --tx_username all-up-tx3 --tx_ip 192.168.1.23 --packet_len 61 --robot locobot4 --ts 5

# On PC: for LOCOBOT5
python ~/Harvard_CS286/cs286_mini_hack_2/scripts/online_data_test2.py --robot_username all-locobot5 --robot_ip 192.168.1.15 --tx_username all-up-tx1 --tx_ip 192.168.1.54 --packet_len 57 --robot locobot5 --ts 5

# On ROBOT for LOCOBOT3
roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100 use_nav:=true use_lidar:=true rtabmap_args:=-d robot_name:=locobot3

# On ROBOT for LOCOBOT4
roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100 use_nav:=true use_lidar:=true rtabmap_args:=-d robot_name:=locobot4

# On ROBOT for LOCOBOT5
roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100 use_nav:=true use_lidar:=true rtabmap_args:=-d robot_name:=locobot5

# On PC for LOCOBOT3 (Optional)
roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot3/map robot_name:=locobot3

# On PC for LOCOBOT4 (Optional)
roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot4/map robot_name:=locobot4

# On PC for LOCOBOT5 (Optional)
roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot5/map robot_name:=locobot5

# On ROBOT for LOCOBOT3
python ~/Harvard_CS286/cs286_mini_hack_2/scripts/collect_robot_displacement_group5.py --position_topic /locobot3/mobile_base/odom --velocity_topic /locobot3/mobile_base/commands/velocity --robot locobot3 --motion curved

# On ROBOT for LOCOBOT4
python ~/Harvard_CS286/cs286_mini_hack_2/scripts/collect_robot_displacement_group5.py --position_topic /locobot4/mobile_base/odom --velocity_topic /locobot4/mobile_base/commands/velocity --robot locobot4 --motion curved

# On ROBOT for LOCOBOT5
python ~/Harvard_CS286/cs286_mini_hack_2/scripts/collect_robot_displacement_group5.py --position_topic /locobot5/mobile_base/odom --velocity_topic /locobot5/mobile_base/commands/velocity --robot locobot5 --motion curved

# On PC for LOCOBOT3
python ~/Documents/GitHub/multirobot-searchandrescue/waypoint.py --robot_name locobot3

# On PC for LOCOBOT4
python ~/Documents/GitHub/multirobot-searchandrescue/waypoint.py --robot_name locobot4

# On PC for LOCOBOT5
python ~/Documents/GitHub/multirobot-searchandrescue/waypoint.py --robot_name locobot5

# On PC for LOCOBOT3
python ~/Documents/GitHub/multirobot-searchandrescue/searcherFSM.py --robot_index 0

# On PC for LOCOBOT4
python ~/Documents/GitHub/multirobot-searchandrescue/searcherFSM.py --robot_index 1

# On PC for LOCOBOT5
python ~/Documents/GitHub/multirobot-searchandrescue/searcherFSM.py --robot_index 2