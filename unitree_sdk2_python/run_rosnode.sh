# !/bin/bash

# Check if an argument was provided
if [ $# -eq 0 ]; then
    echo "Error: Please provide a network interface argument (e.g., enp118s0)"
    exit 1
fi

INTERFACE=$1

# 创建一个新的 tmux 会话
tmux new-session -d -s unitree_ros -n img_pub

# 在 tmux 会话中运行第一个 Python 程序
tmux send-keys -t unitree_ros:img_pub "python example/go2/front_camera/camera_opencv_ros2.py $INTERFACE
" C-m

# 创建一个新的 tmux 窗口
tmux new-window -t unitree_ros -n  cmd_vel

# 在第二个 tmux 窗口中运行第二个 Python 程序
tmux send-keys -t unitree_ros:cmd_vel "python example/go2/high_level/go2_sport_ros.py $INTERFACE
" C-m

tmux detach -s uniree_ros
