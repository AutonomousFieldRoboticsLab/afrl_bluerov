screen -L -Logfile "/home/afrl/logs/camera_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S camera sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/camera.sh
sleep 5
screen -L -Logfile "/home/afrl/logs/mavros_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S mavros sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/mavros.sh
sleep 5
screen -L -Logfile "/home/afrl/logs/bagrecorde_$(date +"%Y_%m_%d_%I_%M_%p").log" -dm -S record sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/bag_recorder.sh