## :)))))


Just for local changes:

source ~/anaconda3/bin/activate
conda activate nisara_hand_eye
source install/setup.zsh

ros2 run auto-hand-eye-calib auto-hand-eye-calib


ros2 run charuco_calib_server charuco_calib_server --ros-args -r camera_info_topic:=/camera/camera/color/camera_info --param camera_info_topic:=/camera/camera/color/camera_info


ros2 launch lbr_bringup bringup.launch.py model:=iiwa14 sim:=false rviz:=true moveit:=true

ros2 launch realsense2_camera rs_launch.py publish_tf:=false
