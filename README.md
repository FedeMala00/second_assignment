# Research Track Second Assignment
## Federico Malatesta (S4803603)

The assignment involves the implementation of three different nodes used to control a robot in 3D space. 
The nodes are used to provide a goal that can be deleted, return the coordinates of the last entered goal, and calculate the distance to the goal and the average speed of the robot.

## Prerequisites
Two different applications are needed to make the developed 3D environment work:
- rviz
- gazebo

these two should already be installed if you followed [the following procedure for installing ros on ubuntu 22.04](https://2023.aulaweb.unige.it/pluginfile.php/154428/mod_resource/content/0/guide_ROSinstallation_Ubuntu22.txt).

A further element to take into consideration in case the simulation does not work is the `robot2_laser.gazebo` file contained into the `urdf` directory, if the robot does not have the laser visible in the rviz window it's necessary to replace it with the one available [at this link](https://github.com/CarmineD8/assignment_2_2023) still contained into the `urdf`directory.

## Installing and running
The first step is installing xterm (program used to manage multiple screen outputs) using this command in the terminal.
```bash 
sudo apt-get install xterm
```
Then it's possible to clone the repository into the `src` directory that is present in the ros environment. 
```bash
git clone https://github.com/FedeMala00/Research_track_ass2.git
```
Subsequently it's necessary to add the execution permission to all the `.py` files contained into the `scripts` directory, so first of all you need to move to `scripts` directory and then use this command.
```bash
chmod +x *.py
```
At this point you can run the program using the appropriately modified launch file.
```bash
roslaunch second_assignment second_assignment.launch
```
By running the programm three different windows will open: 
- one `rviz` window that allows the user to view the simulated robot model, visualize sensor information from the robot's sensors in our case the laser one
- one `gazebo` window that manages the 3D environment within which the robot will move, in fact you can see the present obstacles
- three different `xterm` windows that allow the output on screen for the three created node:
  1. `node_action_client` that requires the choice of a goal that can be canceled and verifies the arrival at the target via feedback,
  2. `last_target_service` prints the instructions to obtain the last target given to the robot, 
  3. `subscriber_pos_vel` prints the distance between the robot and the target and the average velocity of the robot in the last 10 (chosen parameter) observations.


