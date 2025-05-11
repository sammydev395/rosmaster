#! /bin/bash

###############################################################################
# 1.add Additional startup programs
# start_rosmaster_app
# bash /home/jetson/Rosmaster/rosmaster/start_app.sh
# start app program
# 
# 2.add Additional startup programs
# Resolution
xrandr --output DP-1 --mode 1024x600 --rate 60
# set Resolution when no screen
###############################################################################


sleep 5
# sleep 22
gnome-terminal -- bash -c "export JETSON_MODEL_NAME=JETSON_ORIN_NANO;sudo udevadm trigger;python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py;exec bash"

wait
exit 0
