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
git clone https://github.com/FedeMala00/second_assignment.git
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

## Nodes descriptions 
Brief description of all the implemented nodes.
### `node_action_client`
This node is implemented in order to act as a client to an action server. The action server provides an action of type PlanningAction using the "namespace" /reaching_goal founded by using the command `rostopic list`. 
The client allows the user to input a desired `x` and `y` coordinate. These coordinates are then set as parameters in the ROS parameter server and sent as a goal to the action server. The goal is an instance of `PlanningGoal` and contains the target pose (position and orientation) for the robot, in which the position coordinates are set equal to the prevoius `x` and `y`.

The client also subscribes to the `/odom` topic, which publish messages of type Odometry. The callback function for this subscription, `pub_pos_vel`, extracts the position and velocity information from the Odometry message and publishes it to the `/pos_vel_topic` using a custom message `Pos_vel` that can be visualized into the `msg` directory. 
While waiting for the goal to be achieved, the client continuously prints the latest feedback received from the action server which contains the current pose of the robot. If the robot's position is within a range of 0.5 from the target position, the goal is considered reached.

Furthermore the user has the option to cancel the goal by pressing 'c'. If the goal is reached or cancelled, the user can choose to restart the program by pressing 'r' or quit by pressing 'q'. If the program is restarted, the user can input a new target position.

### `last_target_service`
This node provides a service to retrieve the last target coordinates of a robot. 
The service is advertised at the `/last_target_service` topic and uses the `Last_target` service type from the `second_assignment.srv` package. The service response is of type `Last_targetResponse` which includes the last target `x` and `y` coordinates.

When a request is received, the `service_callback` function is called. This function retrieves the last target coordinates from the ROS parameter server (parameters `/des_pos_x` and `/des_pos_y`) and returns them in a `Last_targetResponse`. 
The main function initializes the node, advertises the service, and then enters a spin loop to keep the node running until it's shut down.
The node also logs a message instructing the user to call the service from the terminal to get the last target coordinates using.
```bash 
rosservice call /last_target_service 
```

### `subscriber_pos_vel`

This node subscribes to a topic, provides a service, and also acts as a client to that service.
The node subscribes to the `/pos_vel_topic` topic, which publishes messages of type `Pos_vel`. The `callback_function` for this subscription calculates the Euclidean distance between the current position (retrieved from the `Pos_vel` message) and a goal position (from ROS parameters). It also calculates the average speed of the robot over a window of recent velocities.

The node provides a service at the `/dist_vel_service` topic. The service is of type `Dist_vel` and when a request is received, it responds with the most recently calculated distance and average speed.
The node also acts as a client to the `/dist_vel_service` service. It continuously calls the service and logs the response (the distance and average speed) to the terminal. The rate of these service calls is controlled by a `Rate` object to ensure they occur at approximately 1 Hz.
The node continues to run and call the service until it is shut down.

## Flowchart of `node_action_client`
Flowchart containing the representation of the code, with:
- actions represented by rectangles
- decisions represented by rhombuses

![Flowchart.drawio.png](https://github.com/FedeMala00/Research_track_ass2/blob/main/Flowchart.drawio.png)

## Comments done with `Sphinx`
Link to the `html` page generated by `Sphinx`: https://fedemala00.github.io/second_assignment/
