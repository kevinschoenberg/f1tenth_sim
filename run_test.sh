#! /bin/bash

counter=1
while [ $counter -le 3 ]
do
    cd
    cd ~/catkin_ws/src/f1tenth_sim

    coeff_step_size=0.1
    new_coeff=$(echo "$counter * $coeff_step_size" | bc -l)
    sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" params.yaml

    sleep 1

    cd
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    terminator -T "RL" -e "cd; cd ~/catkin_ws; source devel/setup.bash; roslaunch f1tenth_simulator simulator.launch" 

    sleep 5

    terminator -T "LN" -e "cd; cd ~/catkin_ws; source devel/setup.bash; rosrun f1tenth_simulator LoggerNode.py"
    terminator -T "DN" -e "cd; cd ~/catkin_ws; source devel/setup.bash; rosrun f1tenth_simulator LSDNode.py"

    sleep 5

    rostopic pub -1 /key std_msgs/String v

    sleep 2

    rostopic pub -1 /key std_msgs/String w

    sleep 12

    terminator -e "killall -9 rosmaster"
    ((counter++))
    sleep 5

done
echo All done
