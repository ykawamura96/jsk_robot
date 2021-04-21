function init_device() {
    echo "initializing device..."
    echo "giving access to /dev/tty0 and /dev/tty1"
    sudo chmod 777 /dev/ttyACM0 && sudo chmod 777 /dev/ttyACM1
    rosrun omni_common initialize_device.sh 
}

function start_master_launch () {
    roslaunch cobotta_tools cobotta_touchubs.launch
}


init_device

start_master_launch
