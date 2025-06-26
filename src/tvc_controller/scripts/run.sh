#!/bin/bash

tmux new-session -d -s px4_session

tmux split-window -t px4_session:0.0 -v
tmux split-window -t px4_session:0.1 -h
tmux split-window -t px4_session:0.0 -h

tmux send-keys -t px4_session:0.0 "cd ~/" C-m
tmux send-keys -t px4_session:0.0 "MicroXRCEAgent udp4 -p 8888" C-m 

tmux send-keys -t px4_session:0.1 "cd ~/PX4-Autopilot/" C-m
tmux send-keys -t px4_session:0.1 "make px4_sitl_default" C-m
tmux send-keys -t px4_session:0.1 "PX4_SYS_AUTOSTART=6002 ./build/px4_sitl_default/bin/px4" C-m

tmux send-keys -t px4_session:0.2 "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" C-m

tmux send-keys -t px4_session:0.3 "cd ~/px4_ws/" C-m
tmux send-keys -t px4_session:0.3 "source install/setup.bash" C-m
tmux send-keys -t px4_session:0.3 "ros2 launch tvc_controller lqr.launch.py" 
tmux attach-session -t px4_session
