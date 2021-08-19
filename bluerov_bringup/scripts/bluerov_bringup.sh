screen -dm -S camera sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/camera.sh
sleep 5
screen -dm -S mavros sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/mavros.sh
sleep 5
screen -dm -S record sh /home/afrl/bluerov_ws/src/afrl_bluerov/bluerov_bringup/scripts/bag_recorder.sh