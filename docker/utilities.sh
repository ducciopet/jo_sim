export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
[ -f /home/ros/glim_install/setup.bash ]  && source /home/ros/glim_install/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

alias reload='source /etc/utilities.sh'
alias no_gpu='__NV_PRIME_RENDER_OFFLOAD=0 __GLX_VENDOR_LIBRARY_NAME='

alias description='ros2 launch jo_description description.launch.py'

alias localization='ros2 launch jo_navigation localization.launch.py use_sim_time:=true'
alias localization_gps='ros2 launch jo_navigation localization_gps.launch.py use_sim_time:=true'
alias visodom='ros2 launch jo_navigation visodom.launch.py use_sim_time:=true'

alias navigation='ros2 launch jo_navigation navigation_local.launch.py rviz:=true use_sim_time:=true'
alias navigation_gps='ros2 launch jo_navigation navigation_gps.launch.py rviz:=true use_sim_time:=true'

alias sim='ros2 launch jo_sim launch_sim.launch.py glim:=true'
alias dual_sim='ros2 launch jo_sim launch_dual_robot.launch.py glim:=true teleop_turtlebot:=true '


# LV-DOT aliases
alias glim_bbox='rviz2 -d $(ros2 pkg prefix jo_sim)/share/jo_sim/rviz/glim_bbox.rviz'
alias detector='ros2 launch onboard_detector run_detector.launch.py'

# when working with bags, we want the odom_pub to be true to have the odometry data in the bag, and we want to use sim time
alias detector_bag='ros2 launch onboard_detector run_detector.launch.py odom_pub:=true'
alias localization_detector='ros2 launch jo_navigation localization_detector.launch.py'

ros2 () {
  if [ "$1" = "bag" ] && [ "$2" = "play" ]; then
    shift 2
    local bag_path="$1"
    shift
    local extra_topics=()
    local extra_flags=()
    for arg in "$@"; do
      if [[ "$arg" == /* ]]; then
        extra_topics+=("$arg")
      else
        extra_flags+=("$arg")
      fi
    done
    command ros2 bag play "$bag_path" \
      --topics \
      /clock \
      /front_camera/camera/color/camera_info \
      /front_camera/camera/color/image_raw \
      /front_camera/camera/depth/camera_info \
      /front_camera/camera/depth/image_rect_raw \
      /velodyne_points \
      /imu/data \
      "${extra_topics[@]}" \
      --read-ahead-queue-size 2000 \
      "${extra_flags[@]}"
  else
    command ros2 "$@"
  fi
}

ros2bagrec () {
  if [ -z "$1" ]; then
    echo "Uso: ros2bagrec NOME_BAG"
    return 1
  fi

  command ros2 bag record \
    --topics \
    /front_camera/camera/color/image_raw \
    /front_camera/camera/depth/image_rect_raw \
    /front_camera/camera/color/camera_info \
    /front_camera/camera/depth/camera_info \
    /velodyne_points \
    /imu/data \
    /clock \
    /odometry/filtered \
    --storage mcap \
    -o "$1"
}

# ros2 bag play indor_20260421_1512_0/ --topics /clock /front_camera/camera/color/camera_info /front_camera/camera/color/image_raw /front_camera/camera/depth/camera_info /front_camera/camera/depth/image_rect_raw /velodyne_points /imu/data --loop
