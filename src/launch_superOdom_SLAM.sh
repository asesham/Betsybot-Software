#!/usr/bin/env bash
# launch_all.sh — Launch multiple ROS 2 launch files in separate terminals.

set -e
set -o pipefail

# ---------- Config ----------
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="/home/betsybot/Betsybot-Software/install/setup.bash"

DELAY_SECONDS=15      # default wait time
INTERACTIVE=true      # if true, ask user before launching each node

# ---------- Helpers ----------
die() { echo "[launch_all] $*" >&2; exit 1; }

open_term() {
  local title="$1"; shift
  local cmd="$*"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$cmd; echo; echo '[$title] running. Ctrl+C to stop.'; exec bash" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$cmd; echo; echo [$title] running; read -p 'Press Enter to close...'" &
  else
    die "No supported terminal emulator found (install gnome-terminal or xterm, etc.)."
  fi
}

# ---------- Source ROS + workspace ----------
set +u
source "$ROS_SETUP"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"
set -u 2>/dev/null || true

# ---------- Commands ----------
declare -a TITLES=(
  "Joystick Drive"
  #"RViz Display (betsybot)"
  "RealSense Camera"
  "Hesai Lidar Driver"
  "IMU Converter"
  "Super Odometry (VLP-16)"
  "Map→Odom Broadcaster"
  "PointCloud→Grid"
  #"RViz Map Saver Panel"
  
)

declare -a CMDS=(
  "ros2 launch master_launch Joystick_drive.launch.py"
  #"ros2 launch betsybot display.launch.py"
  "ros2 launch realsense2_camera rs_launch.py"
  "ros2 launch hesai_ros_driver start.py"
  "cd /home/betsybot/Betsybot-Software/src/SuperOdom/script && python3 imu_converter.py --ros-args -p input_topic:=/camera/camera/imu -p output_topic:=/converted/imu -p target_frame:=imu_link_rep103 -p orientation_semantics:=world_from_sensor"
  "ros2 launch super_odometry vlp_16.launch.py"
  "ros2 launch map_odom map_odom_broadcaster.launch.py"
  "ros2 launch pointcloud_to_grid demo.launch.py topic:=/registered_scan"
  #"ros2 launch mapSaver_panel map_saver_panel.launch.py"
)

# ---------- Launch loop ----------
N=${#CMDS[@]}
for (( i=0; i<N; i++ )); do
  echo "[launch_all] Ready to launch: ${TITLES[$i]}"

  if $INTERACTIVE; then
    read -p "Press ENTER to launch '${TITLES[$i]}' (or type 'skip' to skip): " ans
    if [[ "$ans" == "skip" ]]; then
      echo "[launch_all] Skipping: ${TITLES[$i]}"
      continue
    fi
  else
    echo "[launch_all] Waiting ${DELAY_SECONDS}s..."
    sleep "$DELAY_SECONDS"
  fi

  open_term "${TITLES[$i]}" "${CMDS[$i]}"
done

echo "[launch_all] All nodes dispatched."
