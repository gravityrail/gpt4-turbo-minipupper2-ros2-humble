#!/bin/bash
# Install gpt4_ros2 project
# https://github.com/Hermanye996/gpt4_ros2

# set -x
set -e

trap error_check EXIT SIGHUP SIGINT
function error_check() {
    # Check the exit status of the last command
    if [ $? -ne 0 ]; then
        echo "An error or interrupted occurred. The script will now exit."
        read -n1 -r -p "Press any key to continue..." key
        echo ""
        exit 1
    fi
}

# Install necessary dependencies
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3
sudo apt install -y python3-pip
sudo pip install pysocks
sudo pip install requests
sudo pip install openai 

# Clone & build the repository

if [ -d "$HOME/gpt4_ros2_ws" ]; then
  echo "Removing existing gpt4_ros2_ws repository..."
  rm -rf "$HOME/gpt4_ros2_ws"
fi
mkdir -p ~/gpt4_ros2_ws/src
cd ~/gpt4_ros2_ws/src
git clone https://github.com/Hermanye996/gpt4_ros2.git
cd ..
gpt4_ros2_ws_dir=$(pwd)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

sudo sed -i '#source ~/gpt4_ros2_ws/install/setup.bash#d' ~/.bashrc
echo "source ~/gpt4_ros2_ws/install/setup.bash" >> ~/.bashrc
. ~/gpt4_ros2_ws/install/setup.bash

# Ask user for GPT API_KEY
read -p "Enter your GPT API_KEY: " API_KEY
cd $gpt4_ros2_ws_dir/src/gpt4_ros2/gpt4_ros/gpt4_ros
sudo sed -i "s#<YOUR_API_KEY>#$API_KEY#" gpt_config.py
if [[ $? -eq 0 ]]; then
  echo "Add API_KEY executed successfully!"
else
  exit 1
fi

# print success message and wait for user to press any key to run the server and client
read -p "Press any key to run GPT4_ROS2 server & client." -n1 -s
gnome-terminal --disable-factory -- bash -c 'ros2 run gpt4_ros gpt_ros2_server' &
gnome-terminal --disable-factory -- bash -c 'ros2 run gpt4_ros gpt_ros2_client' &

# Exit the script with a success status code
exit 0
