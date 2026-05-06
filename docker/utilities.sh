export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
[ -f /home/ros/glim_install/setup.bash ]  && source /home/ros/glim_install/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash
[ -f /home/ros/install/setup.bash ]       && source /home/ros/install/setup.bash
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

alias reload='source /etc/utilities.sh'
alias no_gpu='__NV_PRIME_RENDER_OFFLOAD=0 __GLX_VENDOR_LIBRARY_NAME='

alias localization='ros2 launch jo_navigation localization.launch.py use_sim_time:=true'
alias localization_gps='ros2 launch jo_navigation localization_gps.launch.py use_sim_time:=true'
alias visodom='ros2 launch jo_navigation visodom.launch.py use_sim_time:=true'

alias navigation='ros2 launch jo_navigation navigation_local.launch.py rviz:=true use_sim_time:=true'
alias navigation_gps='ros2 launch jo_navigation navigation_gps.launch.py rviz:=true use_sim_time:=true'

alias sim='ros2 launch jo_sim launch_sim.launch.py glim:=true'
alias dual_sim='ros2 launch jo_sim launch_dual_robot.launch.py glim:=true teleop_turtlebot:=true '

alias detection='ros2 launch onboard_detector run_detector.launch.py'
alias localization_detector='ros2 launch jo_navigation localization_detector.launch.py'
alias description='ros2 launch jo_description description.launch.py'

ros2 () {
  if [ "$1" = "bag" ] && [ "$2" = "play" ]; then
    shift 2
    command ros2 bag play "$1" \
      --topics \
      /clock \
      /front_camera/camera/color/camera_info \
      /front_camera/camera/color/image_raw \
      /front_camera/camera/depth/camera_info \
      /front_camera/camera/depth/image_rect_raw \
      /velodyne_points \
      /imu/data \
      --loop --clock
  else
    command ros2 "$@"
  fi
}

# ros2 bag play indor_20260421_1512_0/ --topics /clock /front_camera/camera/color/camera_info /front_camera/camera/color/image_raw /front_camera/camera/depth/camera_info /front_camera/camera/depth/image_rect_raw /velodyne_points /imu/data --loop --clock