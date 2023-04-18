#! /bin/bash

declare -A List_acc
List_acc=(['.1']=1.11 [.2]=1.25 [.3]=1.43 [.4]=1.67 ['.5']=2.0 [.6]=2.5 [.7]=3.33 [.8]=5.0 [.9]=7.50 [1.0]=7.51)
vel_counter=1
start_vel=7.
vel_step_size=0.
coeff_step_size=0.1
coeff_step_start=0.1
while [ $vel_counter -le 1 ]
do
    cd
    cd ~/catkin_ws/src/f1tenth_sim

    new_des=$(echo "$start_vel + ($vel_counter - 1) * $vel_step_size" | bc -l)
    sed -i "s/\bdesired_velocity\b:.*/desired_velocity: $new_des/" lsdParam.yaml

    sed -i "s/\bdesired_velocity\b:.*/desired_velocity: $new_des/" logParam.yaml

    sleep 1
    counter=1
    while [ $counter -le 10 ]
    do
        cd
        cd ~/catkin_ws/src/f1tenth_sim

        new_coeff=$(echo "$coeff_step_start + ($counter - 1) * $coeff_step_size" | bc -l)
        sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" params.yaml

        sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" logParam.yaml

        #Use optimal model on each surface
        #new_a_max=${List_acc[$new_coeff]}

        #Use one model for all surfaces
        new_a_max=${List_acc['1.0']}

        #printf '${List_acc[%s]}=%s\n' "${new_coeff}" "$new_a_max"
        sed -i "s/\bmax_accel\b:.*/max_accel: $new_a_max/" params.yaml

        sed -i "s/\bmax_accel\b:.*/max_accel: $new_a_max/" logParam.yaml

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

        sleep 23

        terminator -e "killall -9 rosmaster"
        ((counter++))
        sleep 5

    done
    ((vel_counter++))
done
# Specify the path to the folder you want to delete
cd ~/catkin_ws/
# Delete the folder and its contents
echo deleting build folder
rm -rf build
# Specify the path to the folder you want to delete
# Delete the folder and its contents
echo deleting devel folder
rm -rf devel

echo All done
