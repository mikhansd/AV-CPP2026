#!/bin/bash
echo "======================================"
echo "  Starting Youyeetoo D300 LiDAR"
echo "======================================"

# Kill any existing processes using the serial port
echo "1. Checking for existing processes on /dev/ttyUSB0..."
sudo fuser -k /dev/ttyUSB0 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Killed existing processes"
else
    echo "   ✓ No processes to kill"
fi

# Wait a moment
sleep 2

# Set permissions
echo "2. Setting permissions on /dev/ttyUSB0..."
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
echo "   ✓ Permissions set"

# Source ROS 2 and workspace
echo "3. Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash
source ~/ldlidar_ws/install/setup.bash
echo "   ✓ Environment sourced"

# Check if LiDAR is connected
echo "4. Checking LiDAR connection..."
if [ -e /dev/ttyUSB0 ]; then
    echo "   ✓ LiDAR detected at /dev/ttyUSB0"
else
    echo "   ✗ ERROR: /dev/ttyUSB0 not found!"
    echo "   Please connect your D300 LiDAR via USB"
    exit 1
fi

# Launch the LiDAR
echo "5. Launching D300 LiDAR driver..."
echo "======================================"
echo "   Product: LDLiDAR_LD19"
echo "   Port: /dev/ttyUSB0"
echo "   Baudrate: 230400"
echo "   Topic: /scan"
echo "   Frame ID: lidar_link"
echo "======================================"

# Launch using ros2 command directly
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD19 \
  -p port_name:=/dev/ttyUSB0 \
  -p port_baudrate:=230400 \
  -p topic_name:=scan \
  -p frame_id:=lidar_link \
  -p laser_scan_dir:=true \
  -p enable_angle_crop_func:=false

echo "======================================"
echo "  LiDAR driver has stopped"
echo "======================================"
