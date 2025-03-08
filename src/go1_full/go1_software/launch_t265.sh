#!/usr/bin/expect -f

set ipaddr 14

send "ssh to: $ipaddr and launch T265 \n"
spawn ssh unitree@192.168.123.$ipaddr

expect {
        -re "The.*(yes/no)?"      {send "yes\r"; exp_continue}
        -re "unitree@192.168.123.$ipaddr's password:" {send "123\r" ;exp_continue}
        eof
    }

send -- "~/Unitree/autostart/camerarosnode/cameraRosNode/kill.sh\r"
send -- "export ROS_MASTER_URI=http://128.178.148.56:11311\r"
send -- "roslaunch realsense2_camera rs_t265.launch camera:=cam_2 serial_no:=204422110466\r"

interact