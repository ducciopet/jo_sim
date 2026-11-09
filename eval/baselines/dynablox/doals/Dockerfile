# DOALS reproduction (phase B): the upstream image plus lidar_undistortion, which the upstream DOALS launch
# (play_doals_data.launch) uses to deskew the Ouster scans with the dataset's /tf. Already cloned by https.rosinstall
# in the base image, only not built there (the CODa/lab runs do not use it). Code untouched.
FROM dynablox-noetic:upstream
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws && catkin build lidar_undistortion && test -d devel/lib/lidar_undistortion"
