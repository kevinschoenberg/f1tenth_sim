#! /bin/bash


vel_counter=1
start_vel=5.
vel_step_size=0.
while [ $vel_counter -le 2 ]
do
    cd
    cd ~/catkin_ws/src/f1tenth_sim

    new_des=$(echo "$start_vel + ($vel_counter - 1) * $vel_step_size" | bc -l)
    sed -i "s/\bdesired_velocity\b:.*/desired_velocity: $new_des/" lsdParam.yaml

    sed -i "s/\bdesired_velocity\b:.*/desired_velocity: $new_des/" logParam.yaml

    sleep 1
    counter=1
    while [ $counter -le 2 ]
    do
        cd
        cd ~/catkin_ws/src/f1tenth_sim

        coeff_step_size=0.9
        coeff_step_start=0.1
        new_coeff=$(echo "$coeff_step_start + ($counter - 1) * $coeff_step_size" | bc -l)
        sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" params.yaml

        sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" logParam.yaml

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
    ((vel_counter++))
done
echo All done
