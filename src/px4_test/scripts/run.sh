#!/bin/bash

tmux new-session -d -s px4_session

tmux split-window -t px4_session:0.0 -v
tmux split-window -t px4_session:0.1 -h
tmux split-window -t px4_session:0.0 -h

tmux send-keys -t px4_session:0.0 "cd ~/" C-m
tmux send-keys -t px4_session:0.0 "MicroXRCEAgent udp4 -p 8888" C-m 

tmux send-keys -t px4_session:0.1 "cd ~/PX4-Autopilot/" C-m
tmux send-keys -t px4_session:0.1 "make px4_sitl gz_x500" C-m

tmux send-keys -t px4_session:0.2 "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" C-m

tmux send-keys -t px4_session:0.3 "cd ~/px4_ws/" C-m
tmux send-keys -t px4_session:0.3 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t px4_session:0.3 "source install/setup.bash" C-m
tmux send-keys -t px4_session:0.3 "ros2 launch px4_test px4_test_launch.py" 
tmux attach-session -t px4_session



# # Start a new tmux session named 'px4_session'
# tmux new-session -d -s px4_session

# # Pane 1: Run MicroXRCEAgent
# tmux send-keys -t px4_session "cd ~/" C-m
# tmux send-keys -t px4_session "MicroXRCEAgent udp4 -p 8888" C-m
# sleep 2

# # Split the window horizontally and run PX4 SITL
# tmux split-window -h -t px4_session
# tmux send-keys -t px4_session "cd ~/PX4-Autopilot/" C-m
# tmux send-keys -t px4_session "make px4_sitl gz_x500" C-m

# # Split the window vertically and run ROS 2-Gazebo bridge in the third pane
# tmux split-window -v -t px4_session
# tmux send-keys -t px4_session "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" C-m

# # Split another window vertically and run ROS2 node in the fourth pane
# tmux split-window -v -t px4_session
# tmux send-keys -t px4_session "cd ~/px4_ws/" C-m
# tmux send-keys -t px4_session "source /opt/ros/humble/setup.bash" C-m
# tmux send-keys -t px4_session "source install/setup.bash" C-m
# tmux send-keys -t px4_session "ros2 launch px4_test px4_test_launch.py"

# # # Split the window vertically and run ROS2 node
# # tmux split-window -v -t px4_session
# # tmux send-keys -t px4_session "cd ~/px4_ws/" C-m
# # tmux send-keys -t px4_session "source /opt/ros/humble/ros/setup.bash" C-m
# # tmux send-keys -t px4_session "source install/setup.bash" C-m
# # tmux send-keys -t px4_session "ros2 run px4_test px4_test" 

# # Attach to the tmux session
# tmux attach-session -t px4_session


# #!/bin/bash

# SESSION="scenario"
# SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# #####
# # Get arguments
# #####
# # getopts: function to read flags in input
# # OPTARG: refers to corresponding values
# while getopts s: flag
# do
#     case "${flag}" in
#         s) SCENARIO=${OPTARG};; 
#     esac
# done

# #####
# # Directories
# #####
# SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
# gestelt_bringup_DIR="$SCRIPT_DIR/.."
# PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../PX4-Autopilot"

# #####
# # Sourcing
# #####
# SOURCE_WS="
# source $SCRIPT_DIR/../../../devel/setup.bash &&
# "

# #####
# # Commands
# #####
# # Start drones with planner modules
# CMD_0="
# roslaunch gestelt_bringup $SCENARIO.launch 
# "

# # Start up rviz
# CMD_1="
# roslaunch --wait gestelt_bringup fake_map_central.launch scenario:=$SCENARIO
# "

# # Start up central bridge and nodes
# # CMD_2="
# # roslaunch --wait gestelt_bringup record_single.launch drone_id:=0
# # "

# # Start up script to send commands
# CMD_3="roslaunch --wait gestelt_bringup scenario_mission.launch scenario:=$SCENARIO"

# if [ "$SESSIONEXISTS" = "" ]
# then 

#     tmux new-session -d -s $SESSION

#     tmux split-window -t $SESSION:0.0 -v
#     tmux split-window -t $SESSION:0.1 -h
#     tmux split-window -t $SESSION:0.0 -h

#     tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
#     tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
#     # tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
#     tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
# fi

# # Attach session on the first window
# tmux attach-session -t "$SESSION:0"
