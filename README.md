## This is UR5 controllers with acados

## BASHRC check
1. gedit ~/.bashrc
2. edit path
3. source ~/.bashrc

## Testing with Real robot
1. robot ip parameters: ip: 192.168.1.2/ mask:255.255.255.0/ gateway:192.168.1.1
2. computer ip parameters: ip: 192.168.1.1 / mask:255.255.255.0/ gateway: 192.168.1.1

## Install packages
1. pip install pyyaml
2. pip install rospkg
3. pip install pandas

## URSIM install
install `ursim-5.9.4.10321232`


## Code generate in matlab
1. install acados
2. in bashrc file add the following lines:
export ACADOS_INSTALL_DIR=/home/robot/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/robot/workspaces/WHOLE_ACADOS/acados_set_term_const/c_generated_code
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/robot/acados/lib
export ACADOS_SOURCE_DIR=/home/robot/acados
3. in matlab run the code
4. Open your workspace in terminal, then run python file `insert_at_end.py`. Your generated files will be relocated to `solver` folder.
5. Copy that solver to your workspace


## Check feasibility
rosrun point_feasibility point_feas
rosrun set_feasibility set_feas

## PTC test
1. ./start-ursim.sh
2. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.2
3. rosrun human_vrep test_human.py
4. rosrun human_vrep human_sim
5. rosrun mpc_low mpc_low_node
6. rosrun mpc_high mpc_high_node
7. rosrun mpc_low one_way.py


## STC test
1. ./start-ursim.sh
2. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.2
3. rosrun human_vrep test_human.py
4. rosrun human_vrep human_sim
5. rosrun mpc_low mpc_low_node
6. rosrun set_node set_node
7. rosrun mpc_low one_way.py


