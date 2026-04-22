source /opt/ros/jazzy/setup.bash
[ -f /home/ros/glim_install/setup.bash ]  && source /home/ros/glim_install/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

alias reload='source /etc/utilities.sh'
alias no_gpu='__NV_PRIME_RENDER_OFFLOAD=0 __GLX_VENDOR_LIBRARY_NAME='

alias localization='ros2 launch jo_navigation localization.launch.py use_sim_time:=true'
alias localization_gps='ros2 launch jo_navigation localization_gps.launch.py use_sim_time:=true'

alias navigation='ros2 launch jo_navigation navigation_local.launch.py rviz:=true use_sim_time:=true'
alias navigation_gps='ros2 launch jo_navigation navigation_gps.launch.py rviz:=true use_sim_time:=true'

alias sim='ros2 launch jo_sim launch_sim.launch.py glim:=true'
alias dual_sim='ros2 launch jo_sim launch_dual_robot.launch.py glim:=true teleop_turtlebot:=true'
