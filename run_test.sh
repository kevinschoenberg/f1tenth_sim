#! /bin/bash

declare -A List_acc
List_acc=([1]=1.26 [2]=1.67 [3]=2.0 [4]=2.5 [5]=3.33 [6]=5.0 [7]=7.51)
#List_acc=([.2]=1.26 [.4]=1.67 [.5]=2.0 [.6]=2.5 [.7]=3.33 [.8]=5.0 [.9]=7.51)
vel_counter=1
start_vel=1.
vel_step_size=1.
coeff_step_size=0.025
coeff_step_start=1.0

model_start=1
model_step_size=1

while [ $vel_counter -le 7 ]
do
    cd
    cd ~/catkin_ws/src/f1tenth_sim

    new_des=$(echo "$start_vel + ($vel_counter - 1) * $vel_step_size" | bc -l)
    sed -i "s/\bdesired_velocity\b:.*/desired_velocity: $new_des/" lsdParam.yaml

    sed -i "s/\bdesired_velocity\b:.*/desired_velocity: $new_des/" logParam.yaml

    sleep 1
    mu_counter=1
    while [ $mu_counter -le 1 ]
    do
        model_counter=1
        while [ $model_counter -le 7 ]
        do
            cd
            cd ~/catkin_ws/src/f1tenth_sim

            new_coeff=$(echo "$coeff_step_start + ($mu_counter - 1) * $coeff_step_size" | bc -l)
            sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" params.yaml

            sed -i "s/\bfriction_coeff\b:.*/friction_coeff: $new_coeff/" logParam.yaml

            #Use optimal model on each surface
            new_model=$(echo "$model_start + ($model_counter - 1) * $model_step_size" | bc -l)
            new_a_max=${List_acc[$new_model]}

            #Use one model for all surfaces
            #new_a_max=${List_acc['7']}

            #printf '${List_acc[%s]}=%s\n' "${new_coeff}" "$new_a_max"
            sed -i "s/\bmax_accel\b:.*/max_accel: $new_a_max/" params.yaml

            sed -i "s/\bmax_accel\b:.*/max_accel: $new_a_max/" logParam.yaml

            sed -i "s/\btarget_velocity\b:.*/target_velocity: $new_des/" logParam.yaml
            sed -i "s/\binitial_model\b:.*/initial_model: $new_model/" logParam.yaml

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
            sleep 5

            ((model_counter++))
        done
        ((mu_counter++))
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
