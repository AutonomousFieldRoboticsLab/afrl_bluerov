screen -L -Logfile "/home/afrl/logs/camera_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S camera sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/camera.sh
sleep 5
screen -L -Logfile "/home/afrl/logs/mavros_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S mavros sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/mavros.sh
sleep 5
screen -L -Logfile "/home/afrl/logs/bagrecorder_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S record sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/bag_recorder.sh
screen -L -Logfile "/home/afrl/logs/tags_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S tags sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/tags.sh
screen -L -Logfile "/home/afrl/logs/imu_complimentary_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S imu sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/imu.sh
screen -L -Logfile "/home/afrl/logs/gui_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S gui sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/gui.sh
screen -L -Logfile "/home/afrl/logs/tf_broadcaster_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S tf_broadcaster sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/tf_broadcaster.sh