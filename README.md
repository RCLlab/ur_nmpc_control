## Nonlinear Model Predictive Control with Set Terminal Constraint for Safe Robot Motion Planning in the Presence of Humans

This repository contains the source code for the experiments and results discussed in the paper titled "Nonlinear Model Predictive Control with Set Terminal Constraint for Safe Robot Motion Planning in the Presence of Humans". The code demonstrates the implementation of the algorithms described and provides tools to replicate our findings.

## Dependencies
To run the code, you need the following:

- NumPy
- rospkg
- pyyaml
- pandas

Install all required packages using the following command:
```bash
pip install -r requirements.txt
```

## Implementation and Testing Environment
The algorithms are implemented and tested using ROS on Ubuntu 20.04 with the real robot. Additionally, before testing on the real robot, they were simulated using the URSim simulator 'ursim-5.9.4.10321232'. For installation instructions for URSim, please follow the guidelines available [here](https://www.universal-robots.com/download/?query=).

## Build and Setup
Build the ROS environment with the following command:
```bash
catkin_make
```

Ensure the ROS environment is sourced correctly by adding the following lines to your `.bashrc` file:
```bash
gedit ~/.bashrc
# Add the following line at the end of the file:
# source /path/to/your/catkin_workspace/devel/setup.bash
source ~/.bashrc
```

## Testing the Algorithms
To test the algorithms on the robot, first configure the TCP/IP parameters for both the robot and your computer

Execute the following steps in different terminals:

1. **Launch Robot Drivers:**
   ```bash
   roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.2
   ```
2. **Test Human Movement Simulation:**
   ```bash
   rosrun human_vrep test_human.py
   ```
3. **Send Human Movement Data to Controller:**
   ```bash
   rosrun human_vrep human_sim
   ```
4. **Run Low-Level Controller:**
   ```bash
   rosrun mpc_low mpc_low_node
   ```
5. **Run PTC Controller:**
   ```bash
   rosrun PTC PTC_node
   ```
6. **Send Commands to the Robot:**
   ```bash
   rosrun mpc_low one_way.py
   ```

To test the STC controller, replace step 5 with:
```bash
rosrun STC set_node
```

## Additional Resources
The human dataset used for experiments, known as AnDyDataset, can be accessed [here](https://andydataset.loria.fr/).

## Contact
For questions and feedback, please reach out to Aigerim Nurbayeva at aigerim.nurbayeva@nu.edu.kz
```
