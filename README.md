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
The first step is installing xterm (program used to manage multiple screen outputs) using this command in the terminal 
<pre><code>sudo apt-get install xterm<code><pre>


